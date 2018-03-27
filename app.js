var ctx, chart, movable;
var fieldCtx;
var time = 0;
var fieldSim;

$(function() {
    movable = new Motor();
    $('#currentPosition').text("0");
    $('#run').click(function() {
        var p = $('#p').val();
        var i = $('#i').val();
        var d = $('#d').val();
        var ffv = $('#ffv').val();
        var ffa = $('#ffa').val();
        var tolerance = $('#tolerance').val();
        var totalTime = $('#time').val();
        var momentum = $('#momentum').val();
        var move = $('#move').val();
        var setpoint = $('#setpoint').val();
        runSimulation(p, i, d, ffv, ffa, move, tolerance, setpoint, totalTime, momentum);
    });

    $('#resetPosition').click(function() {
        movable = new Motor();
        $('#currentPosition').text("0");
    });

    $('#resetValues').click(function() {
        // $(this).closest('form').find("input[type=number]").val("");
        $('#sim-form').submit();
        movable = new Motor();
        $('#currentPosition').text("0");
    });

    $('#stop').click(function() {
        movable.controller = null;
    });

    ctx = $("#myChart").get(0).getContext("2d");
    fieldCtx = $('#field').get(0).getContext("2d");
    createChart();
    createField();
});

function createField(){
    fieldSim = new FieldSimulation(fieldCtx, {x: 0, y: 1.3}, {x: 0.8382, y: 0.8382}, {x: 400, y: 222}, 380/16.4592);
    fieldSim.draw();
};

function createChart() {
    var data = {
        labels: [],
        datasets: [{
            label: "Position",
            strokeColor: "#186b10",
            fill: false,
            borderColor: '#186b10',
            data: []
        }]
    };
    var options = {
        scaleShowGridLines: true,
        animation: {
            duration: 0, // general animation time
        },
        hover: {
            animationDuration: 0, // duration of animations when hovering an item
        },
        responsiveAnimationDuration: 0
    };
    chart = new Chart(ctx, {
        type: 'line',
        data: data, 
        options: options
    })
    // chart = new Chart(ctx).Line(data, options);
    Chart.defaults.global.responsive = true;
    Chart.defaults.global.animation = false;
    Chart.defaults.global.elements.point.radius = 0;
};

function updateChart() {
    chart.data.labels.push(Math.round(time * 100) / 100.0);
    chart.data.datasets.forEach((dataset) => {
        dataset.data.push(movable.getPosition());
    });
    fieldSim.update({x: movable.getPosition(), y: 1.3});
    fieldSim.draw();
    chart.update();
    time += 0.01;
}

function runSimulation(p, i, d, ffv, ffa, moveFactor, tolerance, setpoint, totalTime, momentum) {
    movable.spline = new HermiteSpline(movable.getPosition(), 0, setpoint, 0, totalTime);
    movable.momentum = momentum;
    movable.maxSpeed = moveFactor;
    movable.enable(p, i, d, ffv, ffa, tolerance);
    chart.destroy();
    createChart();
    time = 0;
    move(setpoint);
}


function move(setpoint) {
    if (!movable.onTarget(setpoint)) {
        movable.setSetpoint(setpoint);
        $('#currentPosition').text(movable.getPosition());
        updateChart();
        setTimeout(function() {
            move(setpoint);
        }, 0.01);
    } else {
        $('#currentPosition').text(movable.getPosition());
        updateChart();
        movable.disable();
    }
}

function FieldSimulation(canvasCtx, startingPos, size, fieldSizePixels, pixelToMeterRatio){
    this.position = startingPos;
    this.size = size;
    this.ctx = canvasCtx;
    this.pixelToMeterRatio = pixelToMeterRatio;
    this.fieldSize = fieldSizePixels;

    this.draw = function(){
        this.ctx.clearRect(0, 0, this.fieldSize.x, this.fieldSize.y);
        this.ctx.drawImage(document.getElementById("fieldImg"), 0, 0);
        this.ctx.fillStyle = "rgba(255, 255, 255, 0.25)";
        this.ctx.fillRect(0, 0, this.fieldSize.x, this.fieldSize.y);
        this.ctx.fillStyle = "blue";
        this.ctx.fillRect(this.position.x * pixelToMeterRatio, this.position.y * pixelToMeterRatio, this.size.x * pixelToMeterRatio, this.size.y * pixelToMeterRatio);
    };

    this.update = function(position){
        this.position = position;
    };
};

function Motor(){
    this.spline = null;
    this.startTime = -1;
    this.lastUpdateTime = Date.now() / 1000.0;
    this.maxSpeed = 1;
    this.velocity = 0;
    this.currentPosition = 0;
    this.controller = null;
    this.momentum = 0.995;

    this.enable = function(p, i, d, v, a, tolerance){
        this.controller = new MotionProfileController(p, i, d, v, a, tolerance);
        this.startTime = Date.now() / 1000.0;
        this.lastUpdateTime = this.startTime;
        this.controller.minI = -this.maxSpeed;
        this.controller.maxI = this.maxSpeed;
    };

    this.disable = function(){
        this.controller = null;
    };

    this.getPosition = function(){
        return this.currentPosition;
    };

    this.setOutput = function(value){
        var currentTime = Date.now() / 1000.0;
        var v = value * this.maxSpeed;
        this.velocity = v * (1 - this.momentum) + this.velocity * this.momentum;
        this.currentPosition += this.velocity * (currentTime - this.lastUpdateTime);
        this.lastUpdateTime = currentTime;
    };

    this.setSetpoint = function(setpoint){
        var dt = 0.00001;
        var time = Date.now() / 1000.0 - this.startTime;
        var targetPos = this.spline.calculate(time);
        var targetVelocity = this.spline.calculateVelocity(time);
        var targetAcceleration = this.spline.calculateAcceleration(time);
        if (this.controller !== null)
            this.setOutput(this.controller.calculate(this.getPosition(), targetPos, targetVelocity, targetAcceleration));
    };

    this.onTarget = function(setpoint){
        if (this.controller === null)
            return true;
        return this.controller.isOnTarget(setpoint, this.getPosition());
    };
}

function MotionProfileController(p, i, d, ffv, ffa, tolerance) {
    this.kp = p;
    this.ki = i;
    this.kd = d;
    this.kffv = ffv;
    this.kffa = ffa;

    this.tolerance = tolerance;

    this.prevError = 0;
    this.iState = 0;
    this.maxOutput = 1;
    this.minOutput = -1;
    this.maxI = this.maxOutput;
    this.minI = this.minOutput;

    this.lastTime = -1;

    this.reset = function(){
        this.iState = 0;
        this.prevError = 0;
        this.lastTime = -1;
    };

    this.isOnTarget = function(currentPosition, targetPosition){
        var error = targetPosition - currentPosition;
        return Math.abs(error) <= this.tolerance;
    };

    this.calculate = function(currentPosition, targetPosition, targetVelocity, targetAcceleration){
        var error = targetPosition - currentPosition;

        var pTerm = this.kp * error;
        var dTerm = 0;
        var vTerm = this.kffv * targetVelocity;
        var aTerm = this.kffa * targetAcceleration;

        this.iState += error;
        this.iState = Math.min(Math.max(this.iState, this.minI), this.maxI);
        
        var iTerm = this.ki * this.iState;

        var currentTime = Date.now() / 1000.0;

        if(this.lastTime != -1){
            dTerm = this.kd * ((error - this.prevError) / (currentTime - this.lastTime) - targetVelocity);
        }

        var value = pTerm + iTerm + dTerm + vTerm + aTerm;

        this.lastTime = currentTime;
        this.prevError = error;

        return Math.min(Math.max(value, this.minOutput), this.maxOutput);
    };   

}

function HermiteSpline(p0, v0, p1, v1, totalTime){
    this.p0 = p0;
    this.v0 = v0;
    this.p1 = p1;
    this.v1 = v1;
    this.totalTime = totalTime;

    this.calculate = function(time){
        time /= this.totalTime;
        if(time >= 1){
            time = 1;
        } else if (time <= 0){
            time = 0;
        }
        var first = (2 * Math.pow(time, 3) - 3 * Math.pow(time, 2) + 1) * this.p0;
        var second = (Math.pow(time, 3) - 2 * Math.pow(time, 2) + time) * this.v0;
        var third = (-2 * Math.pow(time, 3) + 3 * Math.pow(time, 2)) * this.p1;
        var fourth = (Math.pow(time, 3) - Math.pow(time, 2)) * this.v1;
        return first + second + third + fourth;
    }

      this.calculateVelocity = function(time){
        time /= this.totalTime;
        if(time >= 1){
            time = 1;
        } else if (time <= 0){
            time = 0;
        }
        var first = (6 * Math.pow(time, 2) - 6 * time) * this.p0;
        var second = (3 * Math.pow(time, 2) - 4 * time + 1) * this.v0;
        var third = (-6 * Math.pow(time, 2) + 6 * time) * this.p1;
        var fourth = (3 * Math.pow(time, 2) - 2 * time) * this.v1;
        return first + second + third + fourth;
    };

    this.calculateAcceleration = function(time){
        time /= this.totalTime;
        if(time >= 1){
            return 0;
        } else if (time <= 0){
            return 0;
        }
        var first = (12 * time - 6) * this.p0;
        var second = (6 * time - 4) * this.v0;
        var third = (-12 * time + 6) * this.p1;
        var fourth = (6 * time - 2) * this.v1;
        return first + second + third + fourth;
    };
}


var ctx, chart, movable;
var time = 0;

$(function() {
    movable = new Subsystem();
    $('#currentPosition').text("0");
    $('#run').click(function() {
        var p = $('#p').val();
        var i = $('#i').val();
        var d = $('#d').val();
        var ffv = $('#ffv').val();
        var ffa = $('#ffa').val();
        var tolerance = $('#tolerance').val();
        var totalTime = $('#time').val();
        var move = $('#move').val() / 1000.0 * 20;
        var setpoint = $('#setpoint').val();
        runSimulation(p, i, d, ffv, ffa, move, tolerance, setpoint, totalTime);
    });

    $('#resetPosition').click(function() {
        movable = new Subsystem();
        $('#currentPosition').text("0");
    });

    $('#resetValues').click(function() {
        // $(this).closest('form').find("input[type=number]").val("");
        $('#sim-form').submit();
        movable = new Subsystem();
        $('#currentPosition').text("0");
    });

    $('#stop').click(function() {
        movable.pidController = null;
    });

    ctx = $("#myChart").get(0).getContext("2d");
    createChart();
});

function createChart() {
    var data = {
        labels: [],
        datasets: [{
            label: "Position",
            fillColor: "rgba(0, 0, 0, 0)",
            strokeColor: "#186b10",
            pointColor: "#186b10",
            pointStrokeColor: "#fff",
            pointHighlightFill: "#fff",
            pointHighlightStroke: "#186b10",
            data: []
        }]
    };
    var options = {
        scaleShowGridLines: true,
        bezierCurve: true
    };
    chart = new Chart(ctx).Line(data, options);
    Chart.defaults.global.responsive = true;
    Chart.defaults.global.animation = false;
};

function updateChart() {
    chart.addData([movable.returnPIDInput()], time);
    time += 0.01;
}

function runSimulation(p, i, d, ffv, ffa, moveFactor, tolerance, setpoint, totalTime) {
    movable.spline = new HermiteSpline(movable.position, 0, setpoint, 0, totalTime);
    movable.setMoveFactor(moveFactor);
    movable.enable(p, i, d, ffv, ffa);
    movable.setAbsoluteTolerance(tolerance);
    chart.destroy();
    createChart();
    time = 0;
    move(setpoint);
}


function move(setpoint) {
    if (!movable.onTarget(setpoint)) {
        movable.setSetpoint(setpoint);
        $('#currentPosition').text(movable.returnPIDInput());
        updateChart();
        setTimeout(function() {
            move(setpoint);
        }, 0.01);
    } else {
        $('#currentPosition').text(movable.returnPIDInput());
        updateChart();
        movable.disable();
    }
}


function Subsystem() {
    return {
        position: 0,
        pidController: null,
        moveFactor: 1,
        spline: null,
        lastTime: Date.now() / 1000.0,
        setAbsoluteTolerance: function(tolerance) {
            if (this.pidController === null)
                return;
            this.pidController.tolerance = tolerance;
        },
        setMoveFactor: function(factor) {
            this.moveFactor = factor;
        },
        returnPIDInput: function() {
            return this.position;
        },
        usePIDOutput: function(value) {
            var currentTime = Date.now() / 1000.0;
            this.position += value * this.moveFactor;// * (currentTime - this.lastTime);
            this.lastTime = currentTime;
        },
        enable: function(p, i, d, v, a) {
            this.pidController = new MotionProfileController(p, i, d, v, a, 0.001);
        },
        disable: function() {
            this.pidController = null;
        },
        setSetpoint: function(setpoint) {
            var targetPos = this.spline.calculate(time);
            var targetVelocity = (targetPos - this.spline.calculate(time - 0.01)) / 0.01;
            var targetVelocityNext = (this.spline.calculate(time + 0.01) - targetPos) / 0.01;
            var targetAcceleration = (targetVelocityNext - targetVelocity) / 0.02;
            if (this.pidController !== null)
                this.usePIDOutput(this.pidController.calculate(this.returnPIDInput(), targetPos, targetVelocity, targetAcceleration));
        },
        onTarget: function(setpoint) {
            if (this.pidController === null)
                return true;
            return this.pidController.isOnTarget(setpoint, this.returnPIDInput());
        },
        gaussNoise: function(mean, sigma) {
            var gaussianConstant = 1 / Math.sqrt(2 * Math.PI);
            var x = 0;
            return gaussianConstant * Math.exp(-.5 * x * x) / sigma;
        }
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
}


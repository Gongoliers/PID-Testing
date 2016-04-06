var ctx, chart, movable;
var time = 0;

$(function() {
  movable = new Subsystem();
  $('#currentPosition').text("0");
  $('#run').click(function(){
    var p = $('#p').val();
    var i = $('#i').val();
    var d = $('#d').val();
    var tolerance = $('#tolerance').val();
    var move = $('#move').val();
    var setpoint = $('#setpoint').val();
    runSimulation(p, i, d, move, tolerance, setpoint);
  });

  $('#resetPosition').click(function(){
    movable = new Subsystem();
    $('#currentPosition').text("0");
  });

  $('#resetValues').click(function(){
    // $(this).closest('form').find("input[type=number]").val("");
    $('#sim-form').submit();
    movable = new Subsystem();
    $('#currentPosition').text("0");
  });

  ctx = $("#myChart").get(0).getContext("2d");
  createChart();
});

function createChart(){
  var data = {
    labels: [],
    datasets: [
        {
            label: "Position",
            fillColor: "rgba(220,220,220,0.2)",
            strokeColor: "rgba(220,220,220,1)",
            pointColor: "rgba(220,220,220,1)",
            pointStrokeColor: "#fff",
            pointHighlightFill: "#fff",
            pointHighlightStroke: "rgba(220,220,220,1)",
            data: []
        }
    ]
};
var options = {
  scaleShowGridLines: true,
  bezierCurve : true
};
  chart = new Chart(ctx).Line(data, options);
  Chart.defaults.global.responsive = true;
  Chart.defaults.global.animation = false;
};

function updateChart(){
  chart.addData([movable.returnPIDInput()], time);
  time += 20;
}

function runSimulation(p, i, d, moveFactor, tolerance, setpoint){
  movable.setMoveFactor(moveFactor);
  movable.enable(p, i, d);
  movable.setAbsoluteTolerance(tolerance);
  // chart.clear();
  chart.destroy();
  createChart();
  time = 0;
  // while(!movable.onTarget()){
  //   movable.setSetpoint(setpoint);
  //   $('#currentPosition').text(movable.returnPIDInput());
  //
  // }
  // movable.disable();
  move(setpoint);
}


function move(setpoint){
  if(!movable.onTarget()){
    movable.setSetpoint(setpoint);
    $('#currentPosition').text(movable.returnPIDInput());
    updateChart();
    setTimeout(function(){move(setpoint);}, 20);
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
    setAbsoluteTolerance: function(tolerance){
      if(this.pidController === null)
        return;
      this.pidController.setAbsoluteTolerance(tolerance);
    },
    setMoveFactor: function(factor) {
      this.moveFactor = factor;
    },
    returnPIDInput: function() {
      return this.position;
    },
    usePIDOutput: function(value) {
      value = Math.min(1, value);
      value = Math.max(-1, value);
      this.position += value * this.moveFactor;
    },
    enable: function(p, i, d) {
      this.pidController = new PIDController(p, i, d);
    },
    disable: function() {
      this.pidController = null;
    },
    setSetpoint: function(setpoint) {
      if (this.pidController !== null)
        this.usePIDOutput(this.pidController.setSetpoint(setpoint, this.returnPIDInput()));
    },
    onTarget: function(){
      if(this.pidController === null)
        return false;
      return this.pidController.onTarget();
    }
  };
}

function PIDController(p, i, d, input) {
  return {
    p: p,
    i: i,
    d: d,
    iSum: 0,
    lastValue: 0,
    input: input,
    setpoint: 0,
    tolerance: 0,
    setAbsoluteTolerance: function(tolerance){
      this.tolerance = tolerance;
    },
    setSetpoint: function(setpoint, input) {
      this.setpoint = setpoint;
      this.input = input;
      return this.proportional(setpoint) + this.integral(setpoint) + this.differential(setpoint);
    },
    onTarget: function(){
      return Math.abs(this.setpoint - this.input) <= this.tolerance;
    },
    proportional: function(setpoint) {
      return this.p * (setpoint - this.input);
    },
    integral: function(setpoint) {
      this.iSum += setpoint - this.input;
      return this.iSum * this.i;
    },
    differential: function(setpoint) {
      var current = this.d * ((setpoint - this.input) - this.lastValue);
      this.lastValue = setpoint - this.input;
      return current;
    }
  };
}

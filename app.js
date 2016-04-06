$(function() {
  var movable = new Subsystem();
  // TODO: get p, i, d, setpoint, moveFactor
  // TODO: display current position in graph
  movable.setMoveFactor(3);
  movable.setAbsoluteTolerance(2);
  movable.enable(2.1, 0.0004, 3);
  while(!movable.onTarget()){
    movable.setSetpoint(20);
    console.log(movable.returnPIDInput());
  }
  movable.disable();

});

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
      // value = Math.min(1, value);
      // value = Math.max(-1, value);
      this.position += value * this.moveFactor;
    },
    enable: function(p, i, d) {
      this.pidController = new PIDController(p, i, d, this.returnPIDInput, this.usePIDOutput);
    },
    disable: function() {
      this.pidController = null;
    },
    setSetpoint: function(setpoint) {
      if (this.pidController !== null)
        this.pidController.setSetpoint(setpoint);
    },
    onTarget: function(){
      if(this.pidController === null)
        return false;
      return this.pidController.onTarget();
    }
  };
}

function PIDController(p, i, d, input, output) {
  return {
    p: p,
    i: i,
    d: d,
    iSum: 0,
    lastValue: 0,
    input: input,
    output: output,
    setpoint: 0,
    tolerance: 0,
    setAbsoluteTolerance: function(tolerance){
      this.tolerance = tolerance;
    },
    setSetpoint: function(setpoint) {
      this.setpoint = setpoint;
      console.log(setpoint);
      this.output(this.proportional(setpoint) + this.integral(setpoint) + this.differential(setpoint));
    },
    onTarget: function(){
      return Math.abs(this.setpoint - this.input()) <= this.tolerance;
    },
    proportional: function(setpoint) {
      return this.p * (setpoint - this.input());
    },
    integral: function(setpoint) {
      this.iSum += setpoint - this.input();
      return this.iSum * this.i;
    },
    differential: function(setpoint) {
      var current = this.d * ((setpoint - this.input()) - this.lastValue);
      this.lastValue = setpoint - this.input();
      return current;
    }
  };
}

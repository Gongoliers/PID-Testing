$(function() {
  var movable = new Subsystem();
  // TODO: get p, i, d, setpoint, moveFactor
  // TODO: display current position in graph
  movable.setMoveFactor(1);
  movable.enable(0.3, 0, 0.04);
  movable.setAbsoluteTolerance(0.5);
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

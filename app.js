$(function() {
  var movable = new Subsystem();
  // TODO: get p, i, d, setpoint, moveFactor
  // TODO: display current position in graph
});

function Subsystem() {
  return {
    position: 0,
    pidController: null,
    moveFactor: 1,
    setMoveFactor: function(factor) {
      this.moveFactor = factor;
    },
    move: function(value) {
      value = Math.min(1, value);
      value = Math.max(-1, value);
      this.position += value * this.moveFactor;
    },
    returnPIDInput: function() {
      return this.position;
    },
    usePIDOutput: function(output) {
      this.move(output);
    },
    enable: function(p, i, d) {
      pidController = new PIDController(p, i, d, returnPIDInput, usePIDOutput);
    },
    disable: function() {
      pidController = null;
    },
    setSetpoint: function(setpoint) {
      if (pidController !== null)
        pidController.setSetpoint(setpoint);
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
    setSetpoint: function(setpoint) {
      this.output(proportional(setpoint) + integral(setpoint) + differential(setpoint));
    },
    proportional: function(setpoint) {
      return this.p * (setpoint - this.input());
    },
    integral: function(setpoint) {
      iSum += setpoint - this.input();
      return iSum * this.i;
    },
    differential: function(setpoint) {
      var current = this.d * ((setpoint - this.input()) - lastValue);
      lastValue = setpoint - this.input();
      return current;
    }
  };
}

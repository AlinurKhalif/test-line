// PID constants
const Kp = 0.038;
const Ki = 0.1;
const Kd = 0.76;

let integral = 0;
let prevError = 0;
let speed = 100; // Initial speed
const maxSpeed = 255; // Maximum speed
const minSpeed = 50; // Minimum speed

let connected = false;

function calculatePIDControl(leftSensor: number, middleSensor: number, rightSensor: number): number {
    let error = calculateError(leftSensor, middleSensor, rightSensor);

    // Proportional term
    let proportional = Kp * error;

    // Integral term
    integral += error;
    let integralTerm = Ki * integral;

    // Derivative term
    let derivative = Kd * (error - prevError);
    prevError = error;

    // Calculate the PID control signal
    let controlSignal = proportional + integralTerm + derivative;

    // Return the adjusted control signal
    return controlSignal;
}

function adjustMotorsWithPID(leftSensor: number, middleSensor: number, rightSensor: number): void {
    // Calculate the PID control signal
    let pidControl = calculatePIDControl(leftSensor, middleSensor, rightSensor);

    // Adjust the motor speeds based on the PID control signal
    let leftSpeed = speed + pidControl;
    let rightSpeed = speed - pidControl;

    // Ensure speeds are within the limits
    leftSpeed = Math.max(minSpeed, Math.min(maxSpeed, leftSpeed));
    rightSpeed = Math.max(minSpeed, Math.min(maxSpeed, rightSpeed));

    motor.MotorRun(motor.Motors.M1, motor.Dir.CW, leftSpeed);
    motor.MotorRun(motor.Motors.M2, motor.Dir.CW, leftSpeed);
    motor.MotorRun(motor.Motors.M3, motor.Dir.CW, rightSpeed);
    motor.MotorRun(motor.Motors.M4, motor.Dir.CW, rightSpeed);
}

function calculateError(leftSensor: number, middleSensor: number, rightSensor: number): number {
    // Calculate the error signal based on sensor readings
    let position = leftSensor * (-1) + middleSensor * 0 + rightSensor * 1;
    return position;
}

function lineFollower(): void {
    let leftSensor = pins.digitalReadPin(DigitalPin.P16);
    let middleSensor = pins.digitalReadPin(DigitalPin.P15);
    let rightSensor = pins.digitalReadPin(DigitalPin.P14);

    // Debugging: Display sensor values
    basic.showString("L:" + leftSensor + " M:" + middleSensor + " R:" + rightSensor);

    if (leftSensor == 0 && middleSensor == 1 && rightSensor == 0) {
        // Follow a straight line
        adjustMotorsWithPID(leftSensor, middleSensor, rightSensor);
    } else if (leftSensor == 1 && middleSensor == 0 && rightSensor == 1) {
        // Follow a curved line (adjust motor control for a curve)
        adjustMotorsWithPID(leftSensor, middleSensor, rightSensor);
    } else if (leftSensor == 1 && middleSensor == 0 && rightSensor == 0) {
        // Adjust for left curve
        adjustMotorsWithPID(leftSensor, middleSensor, rightSensor);
    } else if (leftSensor == 0 && middleSensor == 0 && rightSensor == 1) {
        // Adjust for right curve
        adjustMotorsWithPID(leftSensor, middleSensor, rightSensor);
    } else {
        // No line detected, stop all motors
        motor.MotorRun(motor.Motors.M1, motor.Dir.CW, 0);
        motor.MotorRun(motor.Motors.M2, motor.Dir.CW, 0);
        motor.MotorRun(motor.Motors.M3, motor.Dir.CW, 0);
        motor.MotorRun(motor.Motors.M4, motor.Dir.CW, 0);
    }
}

basic.forever(function () {
    // Continuous operation
    if (connected) {
        lineFollower();
    }
    // Additional logic...
});

bluetooth.onBluetoothConnected(function () {
    // Handle Bluetooth connection
    connected = true;
    basic.showLeds(`
        # . . . #
        # . # . #
        # # # # #
        # . . . #
        # . . . #
    `);
});

bluetooth.onBluetoothDisconnected(function () {
    // Handle Bluetooth disconnection
    connected = false;
    basic.showIcon(IconNames.Sad);
});

input.onButtonPressed(Button.A, function () {
    // Button A pressed, move forward
    adjustMotorsWithPID(1, 1, 1);  // Assuming middle sensor is active for a straight line
});

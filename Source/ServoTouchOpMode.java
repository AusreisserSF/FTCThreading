/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.auto;

import android.os.SystemClock;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.ftcdevcommon.Threading;

import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;

// This autonomous OpMode combines the SwivelServoOpMode in
// the current package with the teleop SensorDigitalTouch OpMode
// (converted to autonomous) from the Android Studio project
// FtcFreightFrenzyPB-ND. The file system path is:
// C:\FtcFreightFrenzyPB-ND\FtcRobotController-master\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\teleop\sample
//
// This OpMode first runs the servo and the touch sensor sequentially
// then uses instances of the Java 8 Callable<T> and CompletableFuture<T>
// to run the servo and touch sensor in parallel.
@Autonomous(name = "ServoTouch", group = "Auto")
//@Disabled
public class ServoTouchOpMode extends LinearOpMode {

    private SwivelServo swivelServo;
    private static final int SERVO_CYCLE_MS = 25; // period of each servo cycle

    private RevTouchSensor touchSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        // Get a reference to our servo.
        Servo servo = hardwareMap.get(Servo.class, "swivel_servo");
        swivelServo = new SwivelServo(servo);

        // Get a reference to our touch sensor.
        touchSensor = hardwareMap.get(RevTouchSensor.class, "digital_touch");

        // Wait for the start button.
        telemetry.addData(">", "Press Start");
        telemetry.update();
        waitForStart();

        // Scan servo till time runs out or stop pressed.
        telemetry.addData(">", "Running the servo synchronously");
        telemetry.update();

        try {
            // Move the servo synchronously for 5 seconds.
            makeSwivelServoCallable(5000).call();

            telemetry.addData(">", "Tap the touch sensor 5 times");
            telemetry.update();
            sleep(1000);

            // Monitor the touch sensor until the user presses the button
            // 5 times.
            makeTouchSensorCallable(5).call();

            telemetry.addData(">", "synchronous runs done");
            telemetry.update();
            sleep(1000);

            // Now run the swivel servo and the touch sensor on their own threads.
            RobotLog.d("Running the servo and the touch sensor in parallel");
            telemetry.addData(">", "Running the servo and the touch sensor in parallel");
            telemetry.update();
            sleep(1000);

            // Move the servo *asynchronously* for 5 seconds.
            RobotLog.d("Launching the servo thread");
            swivelServo = new SwivelServo(servo);
            Callable<Double> swivelServoCallable = makeSwivelServoCallable(10000);
            CompletableFuture<Double> swivelServoFuture = Threading.launchAsync(swivelServoCallable);

            // Also run the touch sensor *asynchronously* until the user presses
            // the button 5 times.
            RobotLog.d("Launching the touch sensor thread");
            Callable<Void> touchSensorCallable = makeTouchSensorCallable(5);
            CompletableFuture<Void> touchSensorFuture = Threading.launchAsync(touchSensorCallable);

            // We need both the servo and the touch sensor threads to complete
            // so it's ok to test them in either order.

            RobotLog.d("Wait for the servo thread to complete");
            Double lastServoPosition = Threading.getFutureCompletion(swivelServoFuture);
            RobotLog.d("Servo thread done; last servo position " + lastServoPosition);
            telemetry.addData(">", "servo thread done");
            telemetry.update();
            sleep(1000);

            RobotLog.d("Wait for the touch sensor thread to complete");
            Threading.getFutureCompletion(touchSensorFuture);
            RobotLog.d("Touch sensor thread done");
            telemetry.addData(">", "touch sensor thread done");
            telemetry.update();
            sleep(1000);
        } catch (Exception ex) {
            RobotLog.e(ex.getMessage());
        }
    }

    // Swivel the servo for the requested number of milliseconds.
    // As an illustration, return the position of the servo when time runs out.
    private Callable<Double> makeSwivelServoCallable(int pDurationMilliseconds) {
        Callable<Double> callableSwivelServo = () -> {
            double lastPosition = 0.0;
            ElapsedTime servoTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            servoTimer.reset();
            while (opModeIsActive() && servoTimer.time() < pDurationMilliseconds) {
                lastPosition = swivelServo.moveServo();
                SystemClock.sleep(SERVO_CYCLE_MS);
            }

            return lastPosition;
        };

        return callableSwivelServo;
    }

    // Keep monitoring the touch sensor until the operator presses
    // the button pMaxTouchCount times.
    private Callable<Void> makeTouchSensorCallable(int pMaxTouchCount) {
        return () -> {
            int currentTouchCount = 0;
            boolean waitForRelease = false;
            while (opModeIsActive() && currentTouchCount < pMaxTouchCount) {
                if (!waitForRelease && touchSensor.isPressed()) {
                    waitForRelease = true;
                    telemetry.addData("Touch sensor count", "%2d", ++currentTouchCount);
                    telemetry.update();
                }

                if (waitForRelease && !touchSensor.isPressed())
                    waitForRelease = false;
            }

            return null;
        };
    }
}

Pending review

# Gyro calibration app

Settings → Gyro calibration guides the user through a stationary measurement.
Rest the sensor on a stable surface and press D. The screen shows settling,
measurement progress and the resulting three-axis offset in degrees/second.

D confirms a ready result and saves it. A leaves and discards any unsaved
measurement. Movement or missing data offers a new measurement; an SD error
offers another save attempt using the measured result.

The screen reads status and sends commands to the orientation service's
[bias calibration procedure](../../services/orientation/README.md#gyro-bias-calibration). Its tick
advances that procedure on the shell task. Leaving cancels sampling.

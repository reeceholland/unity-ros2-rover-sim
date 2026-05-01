using UnityEngine;

[System.Serializable]
public class WheelPidController
{
    // PID gains for converting wheel velocity error into WheelCollider torque.
    public float kp = 8f;
    public float ki = 0.5f;
    public float kd = 0f;

    // Limits keep the controller from applying unrealistic torque or winding up while stuck.
    public float outputLimit = 20f;
    public float integralLimit = 10f;

    // Deadband avoids tiny sign-flipping corrections near the target velocity.
    public float deadband = 0.05f;

    // Filtering and slew limiting reduce oscillation from noisy WheelCollider rpm feedback.
    public float measurementFilter = 0.25f;
    public float outputSlewRate = 30f;

    float integral;
    float previousError;
    float filteredMeasured;
    float previousOutput;
    bool hasPreviousError;
    bool hasFilteredMeasured;

    public float Update(float target, float measured, float deltaTime)
    {
        // Smooth measured velocity before calculating error, otherwise rpm jitter can dominate.
        if (!hasFilteredMeasured)
        {
            filteredMeasured = measured;
            hasFilteredMeasured = true;
        }
        else
        {
            filteredMeasured = Mathf.Lerp(filteredMeasured, measured, measurementFilter);
        }

        float error = target - filteredMeasured;

        if (Mathf.Abs(error) < deadband)
        {
            error = 0f;
        }

        // Clamp integral contribution so a stalled wheel cannot build a huge delayed correction.
        integral += error * deltaTime;
        integral = Mathf.Clamp(integral, -integralLimit, integralLimit);

        float derivative = 0f;
        if (hasPreviousError && deltaTime > 0f)
        {
            derivative = (error - previousError) / deltaTime;
        }

        previousError = error;
        hasPreviousError = true;

        float output = kp * error + ki * integral + kd * derivative;
        output = Mathf.Clamp(output, -outputLimit, outputLimit);

        // Prevent instant torque reversals, which feel like forward/backward chatter.
        if (outputSlewRate > 0f && deltaTime > 0f)
        {
            float maxStep = outputSlewRate * deltaTime;
            output = Mathf.MoveTowards(previousOutput, output, maxStep);
        }

        previousOutput = output;
        return output;
    }

    public void Reset()
    {
        integral = 0f;
        previousError = 0f;
        filteredMeasured = 0f;
        previousOutput = 0f;
        hasPreviousError = false;
        hasFilteredMeasured = false;
    }
}

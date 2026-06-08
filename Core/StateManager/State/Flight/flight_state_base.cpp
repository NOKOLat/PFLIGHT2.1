#include "../state_headers.hpp"
#include <cmath>


StateError FlightStateBase::init(StateContext& context) {

    if (context.cascade_pid_manager) {

        context.cascade_pid_manager->reset();
    }

    context.pid_output.fill(0.0f);
    context.throttle = 0.0f;

    return onInit(context);
}


StateError FlightStateBase::update(StateContext& context) {

    constexpr float RAD_TO_DEG = 57.29577951308232f;
    constexpr float DEG_TO_RAD = 0.017453292519943295f;

    // センサーデータを取得（加速度: m/s², ジャイロ: dps）
    context.imu->GetData(context.accel_data.data(), context.gyro_data.data());

//    printf("Accel: [%.2f, %.2f, %.2f] m/s^2, Gyro: [%.2f, %.2f, %.2f] dps\n",
//            context.accel_data[0], context.accel_data[1], context.accel_data[2],
//            context.gyro_data[0], context.gyro_data[1], context.gyro_data[2]);

    // ジャイロを dps → rad/s に変換
    float gyro_rad[3] = {
        context.gyro_data[0] * DEG_TO_RAD,
        context.gyro_data[1] * DEG_TO_RAD,
        context.gyro_data[2] * DEG_TO_RAD,
    };

    // EKF 更新（加速度は内部で正規化される）
    AttitudeEKF_Update(&context.ekf.value(), context.accel_data.data(), gyro_rad);

    // 推定角度を context に格納（単位: deg）
    context.angle.roll  = AttitudeEKF_GetRoll(&context.ekf.value())  * RAD_TO_DEG;
    context.angle.pitch = AttitudeEKF_GetPitch(&context.ekf.value()) * RAD_TO_DEG;
    context.angle.yaw   = AttitudeEKF_GetYaw(&context.ekf.value())   * RAD_TO_DEG;

//    printf("Roll: %.2f deg, Pitch: %.2f deg, Yaw: %.2f deg\n",
//            context.angle.roll,
//            context.angle.pitch,
//            context.angle.yaw);

    // 派生クラスの処理（throttle・pid_outputをcontextに書き込む）
    StateError err = onUpdate(context);

    if (err != StateError::NONE) {

        return err;
    }

    // ミキシング計算・PWM出力(2ループに1回)
    pwm_tick_ = !pwm_tick_;
    if (pwm_tick_) {

    	// センサーの向き依存の方向修正をマイナスでやる
        context.pwm_manager->mix(context.throttle, context.pid_output[0], -context.pid_output[1], -context.pid_output[2]);
        context.pwm_manager->output();
    }

    return StateError::NONE;
}


StateResult FlightStateBase::evaluateNextState(StateContext& context) {

    return onEvaluateNextState(context);
}

void VISION_PREDICT(void)
{

	static uint32_t blank_times = 0;
	uint8_t i = 0;
	int n_num = 0;

	now_yaw = gimbal_imu_yaw_angle_sum;
	now_pitch = gimbal_motor_pid[GIMBAL_M_PITCH].angle.measure;
	now_imu_pitch = (gimbal.dev->gimbal_imu_sensor->pitch);

	average_add(&gimbal_now_yaw_raw, now_yaw);
	average_add(&gimbal_now_pitch_raw, now_pitch);

	if (gimbal.info->local_mode != GIMBAL_MODE_AUTO)//不开启自瞄时，也开启预测
	{
		//times_before_pre = 0;
		//predict_angle_yaw = now_yaw;
		//predict_angle_pitch = now_pitch;
	}

	if (vision_sensor.info->flag == 1)//识别到的情况
	{

		vision_sensor.info->flag = 0;//清除接收标志，避免没有接收到时连续进入

		if (vision_sensor.info->zhen < 40 && 0)//帧率小于40当作丢失
		{
			target_lost_flag = 1;
		}
		else
		{
			times_before_pre++;
		}

		//一个是修改time_ms
		time_ms = (int)Vision_get_interval(target_lost_flag);

		temp_ms_zhen = average_get(&zhen_time_ms, 2) + average_get(&zhen_time_ms, 3) + average_get(&zhen_time_ms, 1);

		yaw_angle_raw = (-vision_sensor.info->yaw_angle) * vision_y_kp + average_get(&yaw_raw_ms, temp_ms_zhen + temp_i2);//获取前一定帧数的yaw角
		pitch_angle_raw = (-vision_sensor.info->pitch_angle) * vision_p_kp + average_get(&pitch_raw_ms, temp_ms_zhen + 5);
		target_imu_pitch = now_imu_pitch + (-vision_sensor.info->pitch_angle);
		//调试时主要看绝对角度是否有变化，理想情况下，目标不动解算出来的结果应该是不变的
		//绝对角度滤波


		if (target_lost_flag >= 1)//丢失后重装载值，避免角度突变
		{
			for (i = 0; i < vision_angleY_MF.lenth; i++)
			{
				average_add(&vision_angleY_MF, yaw_angle_raw);

			}
			for (i = 0; i < vision_angleP_MF.lenth; i++)
			{
				average_add(&vision_angleP_MF, pitch_angle_raw);
			}
			last_angle = yaw_angle_raw;

			for (i = 0; i < 25; i++)//卡尔曼滤波
			{
				KalmanFilter(&vision_angleY_KF, vision_angleY_MF.aver_num);
			}
		}
		average_add(&vision_angleY_MF, yaw_angle_raw);
		average_add(&vision_angleP_MF, pitch_angle_raw);


		auto_pitch_angle = vision_angleP_MF.aver_num;

		yaw_speed_raw = get_auto_speed(&vision_angleY_MF, last_angle) / time_ms;//先滤波再求速度
		//yaw_speed_raw =  get_auto_speed_V2(yaw_angle_raw)/time_ms;//先求速度再滤波，减少滤波延迟叠加
		pitch_speed_raw = get_auto_speed(&vision_angleP_MF, auto_pitch_angle) / time_ms;

		auto_yaw_angle = KalmanFilter(&vision_angleY_KF, vision_angleY_MF.aver_num);

		//算出速度值后需要加一个低通滤波，防止小陀螺阴人
		yaw_speed_raw = LPF_add(&vision_speedY_LPF, yaw_speed_raw);
		pitch_speed_raw = LPF_add(&vision_speedP_LPF, pitch_speed_raw);


		Predict_Anti_Top_binary_update(auto_yaw_angle);
		//判断是否跳变, 如果跳变了，就给反陀螺留下边界
		if (vision_speedY_LPF.High_flag)
		{
			Predict_Anti_Top_Cal_all(last_angle, yaw_angle_raw);
		}
		last_angle = auto_yaw_angle;

		if (target_lost_flag >= 1)//丢失后重装载0，避免速度突变
		{
			for (i = 0; i < vision_speedY_MF.lenth; i++)
			{
				average_add(&vision_speedY_MF, 0);
			}
			for (i = 0; i < vision_speedP_MF.lenth; i++)
			{
				average_add(&vision_speedP_MF, 0);
			}
		}

		average_add(&vision_speedY_MF, yaw_speed_raw);
		average_add(&vision_speedP_MF, pitch_speed_raw);

		auto_yaw_speed = KalmanFilter(&vision_speedY_KF, vision_speedY_MF.aver_num);
		auto_pitch_speed = KalmanFilter(&vision_speedP_KF, vision_speedP_MF.aver_num);

		//加速度项太小了，乘100倍放大一下
		yaw_accel_raw = 100 * get_auto_accel(&vision_speedY_MF, auto_yaw_speed) / time_ms;
		pitch_accel_raw = 100 * get_auto_accel(&vision_speedP_MF, auto_pitch_speed) / time_ms;

		if (target_lost_flag >= 1)//丢失后重装载值，避免速度突变
		{
			for (i = 0; i < vision_accelY_MF.lenth; i++)
			{
				average_add(&vision_accelY_MF, 0);
			}
			for (i = 0; i < vision_accelY_MF.lenth; i++)
			{
				average_add(&vision_accelP_MF, 0);
			}
		}
		average_add(&vision_accelY_MF, yaw_accel_raw);
		average_add(&vision_accelP_MF, pitch_accel_raw);

		auto_yaw_accel = KalmanFilter(&vision_accelY_KF, vision_accelY_MF.aver_num);
		auto_pitch_accel = KalmanFilter(&vision_accelP_KF, vision_accelP_MF.aver_num);

		target_lost_flag = 0;
		vision_lost_flag = 0;


		//计算自瞄获得的实际距离
		distance_raw = vision_sensor.info->distance;//距离
		if (target_lost_flag >= 1)//丢失后重装载值，避免速度突变
		{
			for (i = 0; i < vision_distance_MF.lenth; i++)
			{
				average_add(&vision_distance_MF, distance_raw);
			}
		}
		average_add(&vision_distance_MF, distance_raw);
		auto_distance = KalmanFilter(&vision_distance_KF, vision_distance_MF.aver_num);
		distance_speed_raw = get_auto_speed(&vision_distance_MF, distance_raw) / time_ms;//距离变化速度
		if (target_lost_flag >= 1)//丢失后重装载值，避免加速度突变
		{
			for (i = 0; i < vision_speedY_MF.lenth; i++)
			{
				average_add(&vision_speedD_MF, 0);
			}
		}
		average_add(&vision_speedD_MF, distance_speed_raw);

		auto_distance_speed = KalmanFilter(&vision_speedD_KF, vision_speedD_MF.aver_num);
		distance_accel_raw = get_auto_accel(&vision_speedD_MF, distance_speed_raw) / time_ms;//远离加速度

		predict_distance = auto_distance;


	}
	else if (vision_sensor.info->flag == 2 || (vision_sensor.info->flag == 0 && vision_sensor.info->identify_target == 1))//识别更新间隔
	{

		vision_lost_flag++;

		if (vision_lost_flag > 200)//失联
		{
			vision_sensor.info->identify_target = 0;
		}
		average_add(&vision_distance_MF, distance_raw);

	}
	else if (vision_sensor.info->flag == 0 && vision_sensor.info->identify_target == 0)//目标丢失
	{
		average_clear(&vision_accelY_MF);//清空
		average_clear(&vision_speedY_MF);
		average_clear(&vision_accelP_MF);
		average_clear(&vision_speedP_MF);
		//KalmanFilter(&vision_angleY_KF, now_yaw);//丢失后一直处于跟随当前角度
		//KalmanFilter(&vision_angleP_KF, now_pitch);
		KalmanClear(&vision_speedY_KF);//丢失后清空速度卡尔曼滤波器内容
		LPF_Clear(&vision_speedY_LPF);//低通滤波清理
		LPF_Clear(&vision_speedP_LPF);
		//KalmanFilter(&vision_dis_pitch_offset, 60.0f);//不更新或许更好一点
		//auto_pitch_angle = KalmanFilter(&vision_angleP_KF, now_pitch);//丢失后一直处于跟随当前角度
		times_before_pre = 0;
		target_lost_flag += 1;
	}

	predict_angle_yaw = auto_yaw_angle;//每时更新，绝对角度,这样就相当与每次只增加一次距离
	predict_angle_pitch = auto_pitch_angle;//每时更新，绝对角度



	//根据弹丸速度（也可以说是弹丸飞行时间）修改自瞄预测距离的大小
	if (Friction_speed < 4000)
	{

		if (auto_distance > 4000)
		{
			auto_distance = 4000;
		}

		auto_yaw_speed_kp = (auto_distance * -0.1124f - 244.67f) * 1.5f;//9m/s 公式
		auto_yaw_speed_kp *= 0.4f;


	}
	else if (Friction_speed > 4000)
	{
		auto_yaw_speed_kp = auto_distance * -0.1124f - 244.67f;//14m/s 公式
		auto_yaw_speed_kp *= 0.5f;

		//auto_yaw_speed_kp = (auto_distance * -0.1124f-244.67f)*0.6f;//看哪个准一点
		//auto_yaw_speed_kp *= 0.4f;
	}

	Auto_yaw_speed_offset = auto_yaw_speed_kp * auto_yaw_speed;
	Auto_yaw_accel_offset = auto_yaw_accel * auto_yaw_accel_kp;
	Auto_pitch_speed_offset = auto_pitch_speed * auto_pitch_speed_kp;
	Auto_pitch_accel_offset = auto_pitch_accel * auto_pitch_accel_kp;

	//只有自瞄比较稳定的时候才加预测
	if (times_before_pre > times_before)
	{
		//feed_pre_distance = auto_distance_speed_kp*auto_distance_speed; 
		feed_pre_distance = 0;

		//预测斜坡
		feed_pre_angle_yaw = RampFloat(feed_pre_angle_yaw, Auto_yaw_speed_offset + Auto_yaw_accel_offset, 3);
		feed_pre_angle_pitch = RampFloat(feed_pre_angle_pitch, Auto_pitch_speed_offset + Auto_pitch_accel_offset, 3);//超前预测角
		//预测不斜坡
		//feed_pre_angle_yaw = Auto_yaw_speed_offset + Auto_yaw_accel_offset;
		//feed_pre_angle_pitch = Auto_pitch_speed_offset + Auto_pitch_accel_offset;//超前预测角

		feed_pre_angle_yaw = Auto_yaw_speed_offset + Auto_yaw_accel_offset;
		feed_pre_angle_pitch = Auto_pitch_speed_offset + Auto_pitch_accel_offset;

		if (feed_pre_angle_yaw > 300)//预测量限幅
		{
			feed_pre_angle_yaw = 300;
		}
		else if (feed_pre_angle_yaw < -300)
		{
			feed_pre_angle_yaw = -300;
		}
	}
	else
	{
		feed_pre_angle_yaw = 0.0f * (Auto_yaw_speed_offset + Auto_yaw_accel_offset);
		feed_pre_angle_pitch = 0.0f * (Auto_pitch_speed_offset + Auto_pitch_accel_offset);
		feed_pre_distance = 0;
	}

	//根据对方远离我们的速度进行一个仰角预测

	if (predict_distance + feed_pre_distance < 1500)
	{
		predict_distance = predict_distance;
	}
	else
	{
		predict_distance += feed_pre_distance;
	}

	//抬头角
	//根据射速选择不同的距离俯仰角修正策略
	//低射速的时候弹丸的俯仰角变化较大，高射速的时候俯仰角变化较小
	//根据队里要求的命中率，10m/s射速上限（实际射击速度9.2±0.5m/s）在自瞄情况下不应该大于4m. 
	//在大于4m的时候，射速对俯仰角的影响非常大。目前摩擦轮以及枪管做不到低于±0.5m/s的精度，希望有待提高
	//并且由于仰角遮挡视线的原因，该档射速在大于5m开外的水平地面目标，装甲板已经贴近视觉视野的下方。

	//根据队里要求的命中率，16m/s射速上限（实际射击速度15.0±0.5m/s）
	//在自瞄情况下可以达到视觉识别距离上限（因为灯条太小而识别不到，反馈距离为10m，实际距离为9m），其命中率为80%

	//特别注意的是，在过于贴近的时候，俯仰角必须加大，否则会被反弹击中自己
	dis_to_pitch_offset = predict_cal_shoot_angle(predict_distance, predict_angle_pitch, Friction_speed);
	KalmanFilter(&vision_dis_pitch_offset, dis_to_pitch_offset);

	if (vision_dis_pitch_offset.X_now > 180)//抬头量最多设置180
	{
		predict_angle_pitch = KalmanFilter(&vision_absolute_Pitch_Kal, predict_angle_pitch + feed_pre_angle_pitch) + 180 + predict_err_pitch;
	}
	else
	{
		predict_angle_pitch = KalmanFilter(&vision_absolute_Pitch_Kal, predict_angle_pitch + feed_pre_angle_pitch) + vision_dis_pitch_offset.X_now + predict_err_pitch;
	}




	//predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
	if (IF_KEY_PRESSED_C)
	{
		if (ANTI_TOP_TEST_FLAG == 1)
		{
			//普通预测
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

		}
		else
		{
			//反陀螺
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw * Anti_top_pre_kp;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

			predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
		}

	}
	else
	{
		if (ANTI_TOP_TEST_FLAG == 1)
		{
			//反陀螺
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw * Anti_top_pre_kp;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

			predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
		}
		else
		{
			//普通预测
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

		}
		//predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
	}



	//predict_angle_yaw = constrain(predict_angle_yaw, yaw_angle_raw-300, yaw_angle_raw+300);
	//取出predict，小于整周长的部分
	if (predict_angle_yaw > (GIMBAL_YAW_CIRCULAR_STEP / 2))
	{
		n_num = (int)(predict_angle_yaw / (GIMBAL_YAW_CIRCULAR_STEP));//n圈

		predict_angle_yaw = predict_angle_yaw - n_num * (GIMBAL_YAW_CIRCULAR_STEP);
		if (predict_angle_yaw > GIMBAL_YAW_CIRCULAR_STEP / 2)
		{
			predict_angle_yaw -= GIMBAL_YAW_CIRCULAR_STEP;
		}
	}
	else if (predict_angle_yaw < -(GIMBAL_YAW_CIRCULAR_STEP / 2))
	{
		n_num = (int)(predict_angle_yaw / (GIMBAL_YAW_CIRCULAR_STEP));
		predict_angle_yaw = predict_angle_yaw - n_num * (GIMBAL_YAW_CIRCULAR_STEP);
		if (predict_angle_yaw < -(GIMBAL_YAW_CIRCULAR_STEP / 2))
		{
			predict_angle_yaw += GIMBAL_YAW_CIRCULAR_STEP;
		}
	}

}

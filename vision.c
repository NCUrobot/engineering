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

	if (gimbal.info->local_mode != GIMBAL_MODE_AUTO)//����������ʱ��Ҳ����Ԥ��
	{
		//times_before_pre = 0;
		//predict_angle_yaw = now_yaw;
		//predict_angle_pitch = now_pitch;
	}

	if (vision_sensor.info->flag == 1)//ʶ�𵽵����
	{

		vision_sensor.info->flag = 0;//������ձ�־������û�н��յ�ʱ��������

		if (vision_sensor.info->zhen < 40 && 0)//֡��С��40������ʧ
		{
			target_lost_flag = 1;
		}
		else
		{
			times_before_pre++;
		}

		//һ�����޸�time_ms
		time_ms = (int)Vision_get_interval(target_lost_flag);

		temp_ms_zhen = average_get(&zhen_time_ms, 2) + average_get(&zhen_time_ms, 3) + average_get(&zhen_time_ms, 1);

		yaw_angle_raw = (-vision_sensor.info->yaw_angle) * vision_y_kp + average_get(&yaw_raw_ms, temp_ms_zhen + temp_i2);//��ȡǰһ��֡����yaw��
		pitch_angle_raw = (-vision_sensor.info->pitch_angle) * vision_p_kp + average_get(&pitch_raw_ms, temp_ms_zhen + 5);
		target_imu_pitch = now_imu_pitch + (-vision_sensor.info->pitch_angle);
		//����ʱ��Ҫ�����ԽǶ��Ƿ��б仯����������£�Ŀ�겻����������Ľ��Ӧ���ǲ����
		//���ԽǶ��˲�


		if (target_lost_flag >= 1)//��ʧ����װ��ֵ������Ƕ�ͻ��
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

			for (i = 0; i < 25; i++)//�������˲�
			{
				KalmanFilter(&vision_angleY_KF, vision_angleY_MF.aver_num);
			}
		}
		average_add(&vision_angleY_MF, yaw_angle_raw);
		average_add(&vision_angleP_MF, pitch_angle_raw);


		auto_pitch_angle = vision_angleP_MF.aver_num;

		yaw_speed_raw = get_auto_speed(&vision_angleY_MF, last_angle) / time_ms;//���˲������ٶ�
		//yaw_speed_raw =  get_auto_speed_V2(yaw_angle_raw)/time_ms;//�����ٶ����˲��������˲��ӳٵ���
		pitch_speed_raw = get_auto_speed(&vision_angleP_MF, auto_pitch_angle) / time_ms;

		auto_yaw_angle = KalmanFilter(&vision_angleY_KF, vision_angleY_MF.aver_num);

		//����ٶ�ֵ����Ҫ��һ����ͨ�˲�����ֹС��������
		yaw_speed_raw = LPF_add(&vision_speedY_LPF, yaw_speed_raw);
		pitch_speed_raw = LPF_add(&vision_speedP_LPF, pitch_speed_raw);


		Predict_Anti_Top_binary_update(auto_yaw_angle);
		//�ж��Ƿ�����, ��������ˣ��͸����������±߽�
		if (vision_speedY_LPF.High_flag)
		{
			Predict_Anti_Top_Cal_all(last_angle, yaw_angle_raw);
		}
		last_angle = auto_yaw_angle;

		if (target_lost_flag >= 1)//��ʧ����װ��0�������ٶ�ͻ��
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

		//���ٶ���̫С�ˣ���100���Ŵ�һ��
		yaw_accel_raw = 100 * get_auto_accel(&vision_speedY_MF, auto_yaw_speed) / time_ms;
		pitch_accel_raw = 100 * get_auto_accel(&vision_speedP_MF, auto_pitch_speed) / time_ms;

		if (target_lost_flag >= 1)//��ʧ����װ��ֵ�������ٶ�ͻ��
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


		//���������õ�ʵ�ʾ���
		distance_raw = vision_sensor.info->distance;//����
		if (target_lost_flag >= 1)//��ʧ����װ��ֵ�������ٶ�ͻ��
		{
			for (i = 0; i < vision_distance_MF.lenth; i++)
			{
				average_add(&vision_distance_MF, distance_raw);
			}
		}
		average_add(&vision_distance_MF, distance_raw);
		auto_distance = KalmanFilter(&vision_distance_KF, vision_distance_MF.aver_num);
		distance_speed_raw = get_auto_speed(&vision_distance_MF, distance_raw) / time_ms;//����仯�ٶ�
		if (target_lost_flag >= 1)//��ʧ����װ��ֵ��������ٶ�ͻ��
		{
			for (i = 0; i < vision_speedY_MF.lenth; i++)
			{
				average_add(&vision_speedD_MF, 0);
			}
		}
		average_add(&vision_speedD_MF, distance_speed_raw);

		auto_distance_speed = KalmanFilter(&vision_speedD_KF, vision_speedD_MF.aver_num);
		distance_accel_raw = get_auto_accel(&vision_speedD_MF, distance_speed_raw) / time_ms;//Զ����ٶ�

		predict_distance = auto_distance;


	}
	else if (vision_sensor.info->flag == 2 || (vision_sensor.info->flag == 0 && vision_sensor.info->identify_target == 1))//ʶ����¼��
	{

		vision_lost_flag++;

		if (vision_lost_flag > 200)//ʧ��
		{
			vision_sensor.info->identify_target = 0;
		}
		average_add(&vision_distance_MF, distance_raw);

	}
	else if (vision_sensor.info->flag == 0 && vision_sensor.info->identify_target == 0)//Ŀ�궪ʧ
	{
		average_clear(&vision_accelY_MF);//���
		average_clear(&vision_speedY_MF);
		average_clear(&vision_accelP_MF);
		average_clear(&vision_speedP_MF);
		//KalmanFilter(&vision_angleY_KF, now_yaw);//��ʧ��һֱ���ڸ��浱ǰ�Ƕ�
		//KalmanFilter(&vision_angleP_KF, now_pitch);
		KalmanClear(&vision_speedY_KF);//��ʧ������ٶȿ������˲�������
		LPF_Clear(&vision_speedY_LPF);//��ͨ�˲�����
		LPF_Clear(&vision_speedP_LPF);
		//KalmanFilter(&vision_dis_pitch_offset, 60.0f);//�����»������һ��
		//auto_pitch_angle = KalmanFilter(&vision_angleP_KF, now_pitch);//��ʧ��һֱ���ڸ��浱ǰ�Ƕ�
		times_before_pre = 0;
		target_lost_flag += 1;
	}

	predict_angle_yaw = auto_yaw_angle;//ÿʱ���£����ԽǶ�,�������൱��ÿ��ֻ����һ�ξ���
	predict_angle_pitch = auto_pitch_angle;//ÿʱ���£����ԽǶ�



	//���ݵ����ٶȣ�Ҳ����˵�ǵ������ʱ�䣩�޸�����Ԥ�����Ĵ�С
	if (Friction_speed < 4000)
	{

		if (auto_distance > 4000)
		{
			auto_distance = 4000;
		}

		auto_yaw_speed_kp = (auto_distance * -0.1124f - 244.67f) * 1.5f;//9m/s ��ʽ
		auto_yaw_speed_kp *= 0.4f;


	}
	else if (Friction_speed > 4000)
	{
		auto_yaw_speed_kp = auto_distance * -0.1124f - 244.67f;//14m/s ��ʽ
		auto_yaw_speed_kp *= 0.5f;

		//auto_yaw_speed_kp = (auto_distance * -0.1124f-244.67f)*0.6f;//���ĸ�׼һ��
		//auto_yaw_speed_kp *= 0.4f;
	}

	Auto_yaw_speed_offset = auto_yaw_speed_kp * auto_yaw_speed;
	Auto_yaw_accel_offset = auto_yaw_accel * auto_yaw_accel_kp;
	Auto_pitch_speed_offset = auto_pitch_speed * auto_pitch_speed_kp;
	Auto_pitch_accel_offset = auto_pitch_accel * auto_pitch_accel_kp;

	//ֻ������Ƚ��ȶ���ʱ��ż�Ԥ��
	if (times_before_pre > times_before)
	{
		//feed_pre_distance = auto_distance_speed_kp*auto_distance_speed; 
		feed_pre_distance = 0;

		//Ԥ��б��
		feed_pre_angle_yaw = RampFloat(feed_pre_angle_yaw, Auto_yaw_speed_offset + Auto_yaw_accel_offset, 3);
		feed_pre_angle_pitch = RampFloat(feed_pre_angle_pitch, Auto_pitch_speed_offset + Auto_pitch_accel_offset, 3);//��ǰԤ���
		//Ԥ�ⲻб��
		//feed_pre_angle_yaw = Auto_yaw_speed_offset + Auto_yaw_accel_offset;
		//feed_pre_angle_pitch = Auto_pitch_speed_offset + Auto_pitch_accel_offset;//��ǰԤ���

		feed_pre_angle_yaw = Auto_yaw_speed_offset + Auto_yaw_accel_offset;
		feed_pre_angle_pitch = Auto_pitch_speed_offset + Auto_pitch_accel_offset;

		if (feed_pre_angle_yaw > 300)//Ԥ�����޷�
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

	//���ݶԷ�Զ�����ǵ��ٶȽ���һ������Ԥ��

	if (predict_distance + feed_pre_distance < 1500)
	{
		predict_distance = predict_distance;
	}
	else
	{
		predict_distance += feed_pre_distance;
	}

	//̧ͷ��
	//��������ѡ��ͬ�ľ��븩������������
	//�����ٵ�ʱ����ĸ����Ǳ仯�ϴ󣬸����ٵ�ʱ�����Ǳ仯��С
	//���ݶ���Ҫ��������ʣ�10m/s�������ޣ�ʵ������ٶ�9.2��0.5m/s������������²�Ӧ�ô���4m. 
	//�ڴ���4m��ʱ�����ٶԸ����ǵ�Ӱ��ǳ���ĿǰĦ�����Լ�ǹ�����������ڡ�0.5m/s�ľ��ȣ�ϣ���д����
	//�������������ڵ����ߵ�ԭ�򣬸õ������ڴ���5m�����ˮƽ����Ŀ�꣬װ�װ��Ѿ������Ӿ���Ұ���·���

	//���ݶ���Ҫ��������ʣ�16m/s�������ޣ�ʵ������ٶ�15.0��0.5m/s��
	//����������¿��Դﵽ�Ӿ�ʶ��������ޣ���Ϊ����̫С��ʶ�𲻵�����������Ϊ10m��ʵ�ʾ���Ϊ9m������������Ϊ80%

	//�ر�ע����ǣ��ڹ���������ʱ�򣬸����Ǳ���Ӵ󣬷���ᱻ���������Լ�
	dis_to_pitch_offset = predict_cal_shoot_angle(predict_distance, predict_angle_pitch, Friction_speed);
	KalmanFilter(&vision_dis_pitch_offset, dis_to_pitch_offset);

	if (vision_dis_pitch_offset.X_now > 180)//̧ͷ���������180
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
			//��ͨԤ��
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

		}
		else
		{
			//������
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw * Anti_top_pre_kp;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

			predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
		}

	}
	else
	{
		if (ANTI_TOP_TEST_FLAG == 1)
		{
			//������
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw * Anti_top_pre_kp;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

			predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
		}
		else
		{
			//��ͨԤ��
			predict_angle_yaw = predict_angle_yaw + feed_pre_angle_yaw;
			predict_angle_yaw = KalmanFilter(&vision_absolute_Yaw_Kal, predict_angle_yaw) + predict_err_yaw;

		}
		//predict_angle_yaw = Predict_Anti_Top_Judge_Yaw(predict_angle_yaw);
	}



	//predict_angle_yaw = constrain(predict_angle_yaw, yaw_angle_raw-300, yaw_angle_raw+300);
	//ȡ��predict��С�����ܳ��Ĳ���
	if (predict_angle_yaw > (GIMBAL_YAW_CIRCULAR_STEP / 2))
	{
		n_num = (int)(predict_angle_yaw / (GIMBAL_YAW_CIRCULAR_STEP));//nȦ

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

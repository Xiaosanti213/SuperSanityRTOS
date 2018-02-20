/**
 *
 * @file attitude_estimate.c
 *
 * �Ӽƺ������Ǽ�Ȩ������̬����
 *
 **/
 
 
 #include "attitude_estimate.h" 
 #include <stm32f10x.h>
 #include <math.h>
 #include <stdio.h>
 
 
 
 static void sensors_data_direction_correct(sd*);
 static void euler_to_rotmatrix(const float* euler_delta, float rot_matrix[3][3]);
 static void matrix_multiply(const float mat[3][3], const float* vec1, float* vec2); 
 static void normalize(float*);
 static float fast_inv_sqrt(float); 
 static float dot_product(const float*, const float*);
 static void cross_product(const float* vec1, const float* vec2, float* cp);
 static void rodrigue_rotation_matrix(float* rot_axis, const float rot_angle, float rot_matrix[3][3]);
 static float atan2_numerical(float y, float x);
 static float abs_c_float_version(float);

 
 
 
 
 
/**
 *
 * ���ƣ�sensors_data_direction_correct
 *
 * ���������������ݷ�������
 *
 */
 void sensors_data_direction_correct(sd* sensors_data)
 {
	 float temp;
	 // ��֤��������������XYZ��
	 temp = sensors_data->gyro[0];
	 sensors_data->gyro[0] = sensors_data->gyro[1];
	 sensors_data->gyro[1] = temp;
	 
	 temp = -sensors_data->acc[0];
	 sensors_data->acc[0] = -sensors_data->acc[1];
	 sensors_data->acc[1] = temp;
	 return ;
 }
 
 
 
 
 
/**
 *
 * ���ƣ�attitude_estimate
 *
 * ��������̬����
 *
 */
 void attitude_estimate(ad* attitude_data, sd* sensors_data)
 {
	 float euler_delta[3];						  								//����ֱ�roll pitch yaw
	 float rot_matrix[3][3];						  						  //��ת����2ά����
	 static float att_est[3] = {0,0,1};	  	            //ֻ��ʼ��1��
	 float att_gyro[3];
	 float w_gyro2acc = 0.2;                 							  //��������ԼӼƱ�ֵ
	 u8 i = 0;
	 float deltaT = 0.32;								  							//��������ͨ����������ѭ��ʱ�����ͱȽϺ���
	 
	 sensors_data_direction_correct(sensors_data);      //�������������
     printf_sensors_data_estimate(*sensors_data);       //������ȷȡ�򴫸������ݷ���
	 for(; i < 3; i++)
	 {
	   euler_delta[i] = sensors_data->gyro[i]*deltaT;   //������Ƚ��ϴν������תŷ����
	   attitude_data->angle_rate[i] = sensors_data->gyro[i]; 
	 }
	 euler_to_rotmatrix(euler_delta, rot_matrix);   		//ŷ���Ǽ�����ת����
	 matrix_multiply(rot_matrix, att_est, att_gyro);		//��ǰһ�ι��ƽ�������õ���ǰ��̬ʸ��
	 normalize(sensors_data->acc);				           		//�Ӽ�ʸ��������
	 

	 for (i = 0; i<3; i++)
	 {
	   att_est[i] = (w_gyro2acc*att_gyro[i]+sensors_data->acc[i])/(1+w_gyro2acc);
	 }
	 attitude_data->euler_angle[0] = atan2_numerical(att_est[1],sqrt(att_est[0]*att_est[0]+att_est[2]*att_est[2]));
	 attitude_data->euler_angle[1] = atan2_numerical(att_est[0], att_est[2]);	 
	 printf("Euler   Angle (deg ): %.2f%s%.2f%s%.2f%s",attitude_data->euler_angle[0], "         ",attitude_data->euler_angle[1], "      ",attitude_data->euler_angle[2], "         \n");
	 printf("Angle   Rate  (dps ): %.2f%s%.2f%s%.2f%s",attitude_data->angle_rate[0], "         ",attitude_data->angle_rate[1], "      ",attitude_data->angle_rate[2], "         \n");

	 return ;
 }
 
 

 
 
/**
 *
 * ���ƣ�euler_to_rotmatrix
 *
 * ������ŷ���Ǽ�����ת����
 *
 */
 void euler_to_rotmatrix(const float* euler_delta, float rot_matrix[3][3])
 {
	 float phi = DEG_TO_RAD * euler_delta[0]; //��װ
	 float theta = DEG_TO_RAD * euler_delta[1];
	 float psi = DEG_TO_RAD * euler_delta[2];
	 
	 rot_matrix[0][0] = cos(theta)*cos(psi);
	 rot_matrix[0][1] = cos(theta)*sin(psi);
	 rot_matrix[0][2] = -sin(theta);
	 
	 rot_matrix[1][0] = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
	 rot_matrix[1][1] = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
	 rot_matrix[1][2] = sin(phi)*cos(theta);
	 
	 rot_matrix[2][0] = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
	 rot_matrix[2][1] = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
	 rot_matrix[2][2] = cos(phi)*cos(theta);
	 
	 return;
 }
 
 
 
 
 
 
/**
 *
 * ���ƣ�matrix_multiply
 *
 * ���������������
 *
 */ 
 void matrix_multiply(const float mat[3][3], const float* vec1, float* vec2)//
{
	vec2[0] = mat[0][0]*vec1[0]+mat[0][1]*vec1[1]+mat[0][2]*vec1[2];
    vec2[1] = mat[1][0]*vec1[0]+mat[1][1]*vec1[1]+mat[1][2]*vec1[2];
	vec2[2] = mat[2][0]*vec1[0]+mat[2][1]*vec1[1]+mat[2][2]*vec1[2];
	return ;
}
 
 
 

 
 
/**
 *
 * ���ƣ�normalize
 *
 * ����������������
 *
 */ 
 void normalize(float* vec)
 {
	 float legth = 0;
	 u8 i; 
	 float inv_norm;
	 for(i = 0; i<3; i++)	 					 //Ĭ����ά����
	 {
			legth = vec[i]*vec[i]+legth;
	 }
	 inv_norm = fast_inv_sqrt(legth);//����õ�ģ������֮һ
	 for(i = 0; i<3; i++)
	 {
			vec[i] = vec[i]*inv_norm;    
	 }
 }
 
 
 
 
 
 
/**
 *
 * ���ƣ�fast_inv_sqrt
 *
 * ����������˿��ٿ�ƽ����1/sqrt(num)
 *
 */ 
 float fast_inv_sqrt(float number)
 {
    long i;
	float x2, y;
	const float threehalfs = 1.5f;

	x2 = number * 0.5f;
	y  = number;
	i  = * ( long * ) &y;                       // ����long�Ĵ洢��ʽȡfloat����
	i  = 0x5f3759df - ( i >> 1 );                 
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // һ�ε���
	//	y  = y * ( threehalfs - ( x2 * y * y ) );   // ���ε���
	return y;
 }
 
 
 
 
 
 
/**
 *
 * ���ƣ�calculate_rot_matrix
 *
 * ��������������������ת����
 *
 */ 
 void calculate_rot_matrix(float* vec1, float* vec2, float rot_matrix[3][3])
 {
	 float rot_axis[3];
	 float rot_angle;
	 normalize(vec1);
	 normalize(vec2);
	 rot_angle = acos(dot_product(vec1, vec2));
	 cross_product(vec1, vec2, rot_axis);//���������������������޵������ת�����
	 rodrigue_rotation_matrix(rot_axis, rot_angle, rot_matrix);
	 return ;										
 }
 
 
 
 
 
 
/**
 *
 * ���ƣ�dot_product
 *
 * �����������������
 *
 */ 
 float dot_product(const float* vec1, const float* vec2)
 {
	 u8 i;
	 float dp = 0;
	 for (i = 0; i<3; i++)
	 {
			dp = vec1[i]*vec2[i]+dp;
	 }
	 return dp;
 }
 
 
 
 
/**
 *
 * ���ƣ�cross_product
 *
 * ����������������� 
 *
 */ 
 void cross_product(const float* vec1, const float* vec2, float* cp)
 {
	 cp[0] = vec1[1]*vec2[2]-vec1[2]*vec2[1];
	 cp[1] = vec1[2]*vec2[0]-vec1[0]*vec2[2];
	 cp[2] = vec1[0]*vec2[1]-vec1[1]*vec2[0];
	 return ;
 }
	 
 
 
 
 
 
 
/**
 *
 * ���ƣ�rodrigue_rotation_matrix
 *
 * �������޵������ת��ʽ��������ת������ת�ǣ�������ת����
 *
 */ 
void rodrigue_rotation_matrix(float* rot_axis, const float rot_angle, float rot_matrix[3][3])
{	
  // ��װ
	float w1 = rot_axis[0];
	float w2 = rot_axis[1];
	float w3 = rot_axis[2];
	float cos_theta = cos(rot_angle);
	float sin_theta = sin(rot_angle);
	
	normalize(rot_axis);
	// ���ݹ�ʽ������ת�������Ԫ��
	rot_matrix[0][0] = w1*w1*(1-cos_theta)+cos_theta;
	rot_matrix[0][1] = w1*w2*(1-cos_theta)-w3*sin_theta;
	rot_matrix[0][2] = w1*w3*(1-cos_theta)+w2*sin_theta;
	
	rot_matrix[1][0] = w2*w1*(1-cos_theta)+w3*sin_theta;
	rot_matrix[1][1] = w2*w2*(1-cos_theta)+cos_theta;
	rot_matrix[1][2] = w2*w3*(1-cos_theta)-w1*sin_theta;
	
	rot_matrix[2][0] = w1*w3*(1-cos_theta)-w2*sin_theta;
	rot_matrix[2][1] = w2*w3*(1-cos_theta)+w1*sin_theta;
	rot_matrix[2][2] = w3*w3*(1-cos_theta)+cos_theta;
	
	return ;
}
 
 
 
 
 
 
 
 
/**
 *
 * ���ƣ�rot_matrix_to_euler
 *
 * ��������ת����ŷ����
 *
 */ 
void rot_matrix_to_euler(float R[3][3], float* euler_angle)
{
	
	float theta = atan2_numerical(-R[0][2], 1/fast_inv_sqrt(R[0][0]*R[0][0]+R[0][1]*R[0][1]));
	float psi = atan2_numerical(R[0][1], R[0][0]);
	float phi = atan2_numerical(R[1][2], R[2][2]);
	//��ǰ��λ�ǽǶ�deg
	euler_angle[0] = phi;
	euler_angle[1] = theta;
	euler_angle[2] = psi;
	return ;
}








/**
 *
 * ���ƣ�atan2
 *
 * �������������޼��㷴����
 *
 */ 
float atan2_numerical(float y, float x)     // ��ֹ����
{
	float z = y/x;
	float a;
  if (abs_c_float_version(y) < abs_c_float_version(x))											// 45������Ϊ�ָ���
  {
     a = 57.3 * z / (1.0f + 0.28f * z * z); // �Ȱ���һ�����޼��� 
   if (x<0) 
   {
     if (y<0) a -= 180;
     else a += 180;
   }
  } 
  else
  {
   a = 90 - 57.3 * z / (z * z + 0.28f);
   if (y<0) a -= 180;
  }
  return a;																	// ���ؽǶ�Ϊ��λ
}








/**
 *
 * ���ƣ�abs_c_float_version
 *
 * ���������㸡�������;���ֵ
 *
 */ 
 float abs_c_float_version(float num)	//��c������abs�����������������cpp��abs֧��float����
 {
	 if (num<0)
		return -num;
	else
		return num;
 }







/**
 *
 *  ���ƣ� printf_sensors_data_estimate
 *
 *  ������ ���������������Matlab�Խӣ����Թ������
 *
 */
void printf_sensors_data_estimate(sd sdata)
{
  printf("MPU6050 Accel ( g  ): %.2f%s%.2f%s%.2f%s", sdata.acc[0], "   ", sdata.acc[1], "   ", sdata.acc[2], "   \n");
  printf("MPU6050 Gyro  (dps ): %.2f%s%.2f%s%.2f%s", sdata.gyro[0], "   ", sdata.gyro[1], "    ", sdata.gyro[2], "      \n");
}




 
 
 
 

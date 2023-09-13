/*-------- MCU�̶����� --------*/
#define FOSC 11059200UL //�趨STC8��Ƭ��Ƶ��
#define PWMCycle (60000) //PWM���ڣ���λHz
#define BRT0 (65536 - FOSC/PWMCycle) //��ʱ��0��ֵ�趨������ȷ��PWM����
#define BAUD2 115200 //�趨����2ͨ�Ų�����
#define BRT2 (65536 - FOSC/12/4/BAUD2) //��ʱ��2��ֵ�趨
#define BAUD3 9600 //�趨����3ͨ�Ų�����
#define BRT3 (65536 - FOSC / BAUD3 / 4) //��ʱ��3��ֵ�趨

/*-------- �����̶����� --------*/
#define UARTBufMaxBytes 64 //UART�շ���������ֽ���
#define HallBufMaxBytes 8 //HALL���������ջ�������ֽ���
#define LocationBufMaxBytes 8 //������������ֽ���
#define WheelSize 65 //����ֱ������λmm
#define WheelDistance 175 //2������֮��ľ��룬��λmm

/*-------- ����ٶȻ�PID���� --------*/
#define K_P 120
#define K_I 50
#define K_D 5

/*-------- �˳��ǶȻ�PID���� --------*/
#define K_PA (120 / 1000)
#define K_IA (50 / 1000)
#define K_DA (5 / 1000)

/*-------- �����ɵ����� --------*/
#define DeltaTime 5 //����������ÿ��ʧ�ܵġ���ʱ��
#define MaxOverTime 30 //ÿ�β�������󡰳�ʱʱ�䡱
#define T_Work 20 //����״̬��״̬��������
#define T_Sampling 30 //�������ڣ���λms

#define DeltaDistance 400 //4��ͨ���ľ��������������ֵ�����ݼ��ʱʹ��
#define MaxFailureNum 6 // 3 //������������ʧ�ܴ���������ʱ��ͣ����һ��ʱ��

/*-------- �Ƕȷ������� --------*/
#define StaticDeltaAngle 10 //��̬ת��ʱ����ĽǶȲ�ֵ
#define DynamicDeltaAngle 15 //��̬ת��ʱ����ĽǶȲ�ֵ
#define MinX 100 //С����ת�����СXֵ
#define MinR 600 //�˳�֮�����С����

/*-------- �ٶȲ��� --------*/ 
#define UltraLowSpeed 0.3
#define LowSpeed 0.5
#define TurningSpeed 0.7
#define NormalSpeed 0.8 // 0.6
#define FastSpeed 1.2 // 0.7
#define LowerPWM -1 //�����һ��������͵�PWM
#define ConstPWM -2 //�̶�ռ�ձ�
#define StopPWM -3 //ֹͣ����
#define DeltaDuty 10
#define ConstDuty 150 //�̶�ռ�ձȵ�ֵ

/*-------- �����������ò��� --------*/
//��λ��Ϊmm�����2�����е�����ꣻ������������������ͬһ�߶ȡ�
#define DIS_S 230

/*-------- PWM����ܽ� --------*/
//PWM���������ѹ���ƶ��ܽ�����������Զ���
//PWM0��������:P17��PWM1���ҵ������P16
//����������ʹ���ⲿ�жϣ������Լ�����ܽ�
#define PIN_LGnd P27
#define PIN_RGnd P26

/*-------- ����2/3��������ʹ�õı�־�� --------*/
#define SoundSignalLost 1
#define FindDistance 2
#define FindTemperature 3
#define StartULS 4
#define SoundParameter 5
#define SoundLeft 6
#define SoundRight 7
#define SoundStraight 8
#define SoundStart 9
#define SoundTooFar 10
#define SoundSignalGot 11
#define SoundObstacle 12
#define SoundSpecial 13

/*-------- ����֡����ʱ���õı�־�� --------*/
#define GeneralULSFrame 1
#define TxdULSFrame 2
#define RxdULSFrame 3

/*-------- ͨ��ѭ����ʹ�õı�־�� --------*/
#define WAIT_START 0 //�ȴ�̬������ͨ��ѭ����ʼ��
#define START_TXD 1 //��������������
#define WAIT_TXD_ACK 2 //�ȴ�����������Ӧ��
#define WAIT_READ 3 //��ʱ�ȴ����ն��յ��ź�
#define START_READ 4 //��ʼ��ȡ��������
#define WAIT_READ_ACK 5 //�ȴ���ȡ�����Ӧ��
#define CAL_XYZ 6 //���ݾ������ݼ���XY����
#define CONTROL 7 //��������XY�������������������п���

/*-------- ����max��min���� --------*/
#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )

/*-------- ��ʱ��0����1ms���� --------*/
// ��ʼλ��ֵ = 65536 - 11059200/12*t0 (t0Ϊʱ����λ����1ms��̫�󲻿��ԣ�
#define Timer1msH 0xFC
#define Timer1msL 0x66

/*-------- AUXR�Ĵ��������� --------*/
//��ʱ��0��1��2��ģʽ//
#define AUXR_T0x12 0x00 //T0����12��Ƶ
#define AUXR_T0x1 0x80 //T0������12��Ƶ
#define AUXR_T1x12 0x00 //T1����12��Ƶ
#define AUXR_T1x1 0x40 //T1������12��Ƶ
#define AUXR_T2x12 0x00 //T2����12��Ƶ
#define AUXR_T2x1 0x04 //T2������12��Ƶ
//����ͨѶ//
#define AUXR_UART1_M0x1 0x00 //����1ģʽ0�����ʲ��ӱ�
#define AUXR_UART1_M0x6 0x20 //����1ģʽ0�����ʼӱ���*6��
#define AUXR_UART1_T1 0x00 //��T1�������ʷ�����
#define AUXR_UART1_T2 0x01 //��T2�������ʷ�����

/*-------- SCON�Ĵ��������� --------*/
//����1��ģʽ
#define UART1_M0 0x00
#define UART1_M1 0x40
#define UART1_M2 0x80
#define UART1_M3 0xC0

/*-------- S2CON�Ĵ��������� --------*/
//����2��ģʽ
#define UART2_M0 0x00
#define UART2_M1 0x80

/*-------- S3CON�Ĵ��������� --------*/
//����3��ģʽ��������UART3�����루S3REN = 1��
#define UART3_M0T3 0x50
#define UART3_M1T3 0xD0

/*-------- TMOD�Ĵ��������� --------*/
//��ʱ��0��ģʽ
#define Timer0_M0 0x00 //ģʽ0��16λ�Զ�����
#define Timer0_M1 0x01 //ģʽ1��16λ���Զ�����
#define Timer0_M2 0x02 //ģʽ2��8λ�Զ�����
#define Timer0_M3 0x03 //ģʽ3��ֹͣ����
//��ʱ��1��ģʽ
#define Timer1_M0 0x00 //ģʽ0��16λ�Զ�����
#define Timer1_M1 0x10 //ģʽ1��16λ���Զ�����
#define Timer1_M2 0x20 //ģʽ2��8λ�Զ�����
#define Timer1_M3 0x30 //ģʽ3��ֹͣ����
//��ʱ��2��3��4��ģʽ�̶�Ϊ16λ�Զ�����
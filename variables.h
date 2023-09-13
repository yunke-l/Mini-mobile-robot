#include "constants.h"
#include "stc8.h"

/*-------- ULS�������� --------*/
unsigned char xdata g_ucOverTimeCnt; //���������г�ʱ��
unsigned char xdata g_ucFailureNum; //�������ݴ��󣨵��ߣ��Ĵ���
int xdata ga_iDistance[4]; //4·��������⵽�ľ���
unsigned char xdata g_ucSpcDstNum; //���������ţ���ga_iDistance�е���ţ��������겻ʹ�õľ�������

/*-------- ULS�������� --------*/
float xdata g_fCurX; //��ǰX����
float xdata g_fCurY; //��ǰY����
float xdata g_fCurZ; //��ǰZ����
float xdata g_fCurR; //��ǰ���룬����������
float xdata g_fCurA; //��ǰ�Ƕȣ�����������
float xdata ga_fCurXBuf[LocationBufMaxBytes]; //��Ч��X���껺��
float xdata ga_fCurYBuf[LocationBufMaxBytes]; //��Ч��Y���껺��
unsigned char g_ucLocationSavePst; //�������λ�õ�Ԫ�ر��

/*-------- PID�ٶȿ������� --------*/
float xdata g_fLeftCurError; //�����ٶ����
float xdata g_fLeftLastError; //������һ���ٶ����
float xdata g_fRightCurError; //�ҵ���ٶ����
float xdata g_fRightLastError; //������һ���ٶ����

/*-------- PID�Ƕȿ������� --------*/
float xdata g_fAngleCurError; //��ǰ�Ƕ����
float xdata g_fAngleLastError; //��һ�νǶ����
float xdata g_fAngleLast2Error; //��2�νǶ����
int xdata g_iSign; //����Ϊ1������Ϊ-1

/*-------- �ٶȿ������� --------*/
float xdata g_fLeftCurV; //��ǰ�ٶȣ���λm/s
float xdata g_fLeftSetV; //���õ�Ŀ���ٶȣ���λm/s
float xdata g_fRightCurV; //��ǰ�ٶȣ���λm/s
float xdata g_fRightSetV; //���õ�Ŀ���ٶȣ���λm/s
float xdata g_fSetA; //���õ�Ŀ��Ƕȣ���λ��

/*-------- ��ѭ������ --------*/
unsigned char xdata g_ucWorkStat; //�������ʱ״̬��
unsigned char xdata g_ucTmpFlag; //��ʱFLAG
unsigned char xdata g_ucTmp;
float xdata g_fTmp;

/*-------- ������������ --------*/
bit g_bSingleLostFlag;
bit g_bSoundStartFlag;
bit g_bSoundSpecialFlag;
bit g_bSoundObstacleFlag;

/*-------- �����ϰ������� --------*/
bit g_bForwardFlag;
bit g_bBackwardFlag;
bit g_bLeftwardFlag;
bit g_bRightwardFlag;
bit g_bNoObstacleFlag;

/*-------- ʱ����־�� --------*/
bit g_b1msFlag;
bit g_b10msFlag;
bit g_b50msFlag;
bit g_b100msFlag;
bit g_b500msFlag;
bit g_b1sFlag;
bit g_b3sFlag;
bit g_b5sFlag;
bit g_bSamplingFlag; //��������ʱ��
bit g_bWorkFlag; //����״̬��ʱ��
bit g_b1minFlag;
bit g_b3minFlag;
unsigned int g_uiCnt; //ʱ����������
unsigned char g_uiMinCnt; //���Ӽ�����

/*-------- PWM���� --------*/
float xdata g_uiLeftCurDuty; //������ǰռ�ձ�
float xdata g_uiRightCurDuty; //�ҵ����ǰռ�ձ�

/*-------- �������������� --------*/
unsigned char xdata ga_ucLeftHallRxdBuf[HallBufMaxBytes]; //���������������
unsigned char xdata ga_ucRightHallRxdBuf[HallBufMaxBytes]; //�һ�������������
float g_fLeftHallCurCnt; //��������������0��ǰ��������
float g_fRightHallCurCnt; //�ҵ������������0��ǰ��������
unsigned char g_ucLeftHallCnt; //��������������0��ʱ��������
unsigned char g_ucRightHallCnt; //�ҵ������������0��ʱ��������
unsigned char g_ucHallSavePst; //���ҵ������������������Ԫ�ر��

/*-------- �����շ����ݴ������ --------*/
unsigned char xdata ga_ucUartTxdBuf[UARTBufMaxBytes]; //UART���ͻ�����
unsigned char xdata ga_ucUartRxdBuf[UARTBufMaxBytes]; //UART���ջ�����
unsigned char g_ucUART2SavePst; //����UART2�������ݵ�Ԫ�ر��
bit g_bUart3BusyFlag; //����1������æ
bit g_bUart2BusyFlag; //����2������æ
/*-------- ͷ�ļ� --------*/
#include "stc8.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "intrins.h"
#include "string.h"
#include "variables.h"

/*-------- �������� --------*/
//Ӳ����ʼ��
void HardwareInit();
void Uart2Init();
void Uart3Init();

//�ײ�Ӳ�����ƣ�PWM�Լ�LED�ƣ�
void PWMLeftCon(unsigned int duty);
void PWMRightCon(unsigned int duty);
void HallCntFiltering();
void LedToogle(int n);

//�����ʼ��
void VariablesInit();

//�ײ��������
void GenerateTimeFlag();
void DelayTime(int n);
void ClearRxdBuf(); //���UART���ջ�����

//���䳬�������
void TxdData(int n);
unsigned char CheckUartRxd(); //�����յ�ֵ
bit CheckDistance(); //������
void ClearMeaBuf(); //���ÿ�ֲ�����λ��ֵ
void CalLocation(); //�����˵�λ�ã����꣩
void StoreLocation(); //��ÿһ����Ч����������ֵ���д洢

//���Ƶ���˶�
void LeftVelocityFDBK(float i_fSetV); //�����ٶȷ���
void RightVelocityFDBK(float i_fSetV); //�ҵ���ٶȷ���
void DynamicAngleFDBK(); //��̬�Ƕȷ���
void StaticAngleFDBK(); //��̬�Ƕȷ���

//�������
void CheckObstacle(); //��������谭

//������//
void main()
{
    Uart2Init();
    Uart3Init();
    HardwareInit();
    VariablesInit();
    EA = 1; //�������ж�
    LedToogle(10); //Ϩ��LED��
    TxdData(SoundStart);
    DelayTime(3000);
    while(1)
    {
        /*-------- ʱ������ --------*/
        GenerateTimeFlag();
        //����ʱ���־��
        if(g_b10msFlag)
        {
            g_b10msFlag = 0;
        }
        if(g_bSamplingFlag)
        {
            g_bSamplingFlag = 0;

            g_fLeftHallCurCnt = g_ucLeftHallCnt; //�˲�ǰ���г�ʼ������ֹ���ݳ���
            g_fRightHallCurCnt = g_ucRightHallCnt;
            g_ucLeftHallCnt = 0;
            g_ucRightHallCnt = 0;

//            HallCntFiltering(); //��������������˲�
            //����������
            if(g_bSingleLostFlag)
            {
                g_bSingleLostFlag = 0;
                TxdData(SoundSignalLost);
                g_ucFailureNum = 0;
                DelayTime(2000);
            }
            if(g_ucFailureNum > MaxFailureNum)
            {
                g_fLeftSetV = StopPWM;
                g_fRightSetV = StopPWM;
                g_bSingleLostFlag = 1;
            }
            if(g_bSoundStartFlag)
            {
                g_bSoundStartFlag = 0;
                TxdData(SoundStart);
                DelayTime(3000);
            }
            if(g_bSoundSpecialFlag)
            {
                g_bSoundSpecialFlag = 0;
                TxdData(SoundSpecial);
                DelayTime(3000);
            }
            if(g_bSoundObstacleFlag)
            {
                g_bSoundObstacleFlag = 0;
                TxdData(SoundObstacle);
                DelayTime(800);
            }
            if(g_b1minFlag)
            {
                g_b1minFlag = 0;
                g_fLeftSetV = StopPWM;
                g_fRightSetV = StopPWM;
                if(g_b3minFlag == 0)
                {
                    g_bSoundStartFlag = 1; //���ͻ�ӭ����
                    g_bSoundSpecialFlag = 0;
                }
                else if(g_b3minFlag == 1)
                {
                    g_b3minFlag = 0;
                    g_bSoundSpecialFlag = 1;
                    g_bSoundStartFlag = 0;
                }
            }
            CheckObstacle();
            if(g_bNoObstacleFlag == 0)//���ϰ�
            {
                g_fLeftSetV = StopPWM;
                g_fRightSetV = StopPWM;
                g_bSoundObstacleFlag = 1;
            }

            g_fLeftCurV = g_fLeftHallCurCnt * 3.14 * WheelSize / 11 / 20 / T_Sampling; // ��ĸ���ǿɵ��ڵ�ʱ�����ڣ���50[ms]�����嵥λm/sj; 20�Ǽ��ٱ�
            if(g_fLeftHallCurCnt > 0)
            {
                LedToogle(3);
            }
            LeftVelocityFDBK(g_fLeftSetV);
            if(g_fLeftCurV > 0.95 * g_fLeftSetV && g_fLeftCurV < 1.05 * g_fLeftSetV)
            {
                LedToogle(2);
            }

            g_fRightCurV = g_fRightHallCurCnt * 3.14 * WheelSize / 11 / 20 / T_Sampling; // ��ĸ���ǿɵ��ڵ�ʱ�����ڣ���50[ms]�����嵥λm/sj; 20�Ǽ��ٱ�
            if(g_fRightHallCurCnt > 0)
            {
                LedToogle(4);
            }
            RightVelocityFDBK(g_fRightSetV);

            if(g_fRightCurV > 0.95 * g_fRightSetV && g_fRightCurV < 1.05 * g_fRightSetV)
            {
                LedToogle(5);
            }

//            if(g_bNoObstacleFlag == 0)
//            {
//								g_bNoObstacleFlag = 1;
//                TxdData(SoundObstacle);
//                DelayTime(1000);
//            }
        }
        if(g_b50msFlag)
        {
            g_b50msFlag = 0;
        }
        if(g_b1sFlag)
        {
            g_b1sFlag = 0;
        }
        if(g_b3sFlag)
        {
            g_b3sFlag = 0;
//            TxdData(SoundParameter); //debug
        }
        if(g_b5sFlag)
        {
            g_b5sFlag = 0;
        }
        /*-------- ͨ�Ŵ��� --------*/
        if(g_bWorkFlag)
        {
            g_bWorkFlag = 0;
            //����״̬��������
            switch(g_ucWorkStat)
            {
            case WAIT_START: //�ȴ��µ�һ�ֲ���
            {
                g_ucWorkStat++;
                break;
            }
            case START_TXD: //��������������
            {
                ClearRxdBuf(); //ÿ�η���ָ��ǰ��ջ���
                TxdData(StartULS);
                g_ucOverTimeCnt = 0;
                DelayTime(DeltaTime); //��ʱһ��ʱ�䣬�����źŴ��ݹ���
                g_ucWorkStat++;
                break;
            }
            case WAIT_TXD_ACK: //�ȴ�������T��Ӧ��
            {
                if(CheckUartRxd() == TxdULSFrame)
                {
                    g_ucWorkStat += 2; //����WAIT_READ
                }
                else
                {
                    g_ucOverTimeCnt += DeltaTime;
                    DelayTime(DeltaTime);
                    if(g_ucOverTimeCnt > MaxOverTime)
                    {
                        g_ucWorkStat = WAIT_START; //��ʱ���ص��ȴ�����״̬
                    }
                }
                break;
            }
            case WAIT_READ:
            {
                g_ucWorkStat++;
                break;
            }
            case START_READ:
            {
                ClearRxdBuf();
                TxdData(FindDistance);
                DelayTime(DeltaTime); //��ʱһ��ʱ�䣬�����źŴ��ݹ���
                g_ucWorkStat++;
                break;
            }
            case WAIT_READ_ACK:
            {
                if(CheckUartRxd() == RxdULSFrame)
                {
                    LedToogle(0);
                    g_ucWorkStat++;
                }
                else
                {
                    g_ucOverTimeCnt += DeltaTime;
                    if(g_ucOverTimeCnt > MaxOverTime)
                    {
                        g_ucWorkStat = WAIT_START; //��ʱ���ص��ȴ�����״̬
                    }
                }
                break;
            }
            case CAL_XYZ:
            {
                //�����Ѿ����յ��ľ������ݼ���R�˴��ص�4������
                for(g_ucTmp = 0; g_ucTmp < 4; g_ucTmp++) //������ʵ����
                {
                    ga_iDistance[g_ucTmp] = 256 * ga_ucUartRxdBuf[11 + 2 * g_ucTmp] + ga_ucUartRxdBuf[10 + 2 * g_ucTmp];
                }
                if(CheckDistance()) //������ֵ����Χ��
                {
                    CalLocation();
                    StoreLocation();
                    LedToogle(1); //������ɫLED��
                    g_ucWorkStat++;
                    break;
                }
                else //������ֵ������Χ�����²���
                {
                    g_ucFailureNum++;
                    g_ucWorkStat = WAIT_START;
                    break;
                }
            }
            case CONTROL:
            {
                g_ucFailureNum = 0; //�Ե��ߴ�������
                g_fSetA = 90;
                if(g_bNoObstacleFlag)
                {
                    if(g_fCurR < MinR)
                    {
                        StaticAngleFDBK();
                    }
                    else
                    {
                        DynamicAngleFDBK();
                    }
                    if(g_fCurR > 3 * MinR)
                    {
                        TxdData(SoundTooFar);
                    }
                }
                else if(g_bNoObstacleFlag == 0)//���ϰ�
                {
                    g_fLeftSetV = StopPWM;
                    g_fRightSetV = StopPWM;
                    g_bSoundObstacleFlag = 1;
                }
                ClearMeaBuf();
                g_ucWorkStat = WAIT_START;
                break;
            }
            }
        }
    }
}

/*-------- ��ͨ���� --------*/

/******************************/
/*-------- ������ʼ�� --------*/
/*****************************/
void VariablesInit()
{
    //�ϲ������Ƴ�ʼֵ
    g_fLeftSetV = 0;
    g_fRightSetV = 0;
    g_uiLeftCurDuty = 0;
    g_uiRightCurDuty = 0;
    g_fSetA = 0;
    g_fAngleCurError = 0;
    g_fAngleLastError = 0;

    //�ײ������Ƴ�ʼֵ
    g_ucLeftHallCnt = 0;
    g_ucRightHallCnt = 0;
    g_fLeftCurError = 0;
    g_fLeftLastError = 0;
    g_fRightCurError = 0;
    g_fRightLastError = 0;

    g_ucTmp = 0; //��ʱ����
    g_uiCnt = 0;

    //ʱ����־��//
    g_b1msFlag = 0;
    g_b1sFlag = 0;
    g_b3sFlag = 0;
    g_bWorkFlag = 0;
    g_bSamplingFlag = 0;
    g_uiMinCnt = 0;
    g_uiCnt = 0;
    g_b1minFlag = 0;
    g_b3minFlag = 0;

    //�����շ���־��//
    g_ucUART2SavePst = 0;
    g_ucWorkStat = 0;
    g_bUart2BusyFlag = 0;
    g_bUart3BusyFlag = 0;

    //��ʼ������
    for(g_ucTmp = 0; g_ucTmp < 4; g_ucTmp++)
    {
        ga_iDistance[g_ucTmp] = 0;
    }
    g_ucLocationSavePst = 0;
    g_ucFailureNum = 0;

    //��ʼ������
    g_fCurX = 0;
    g_fCurY = 0;
    g_fCurZ = 0;
    g_fCurA = 90;
    g_fCurR = 0;

    //��ʼ����������
    for(g_ucTmp = 0; g_ucTmp < HallBufMaxBytes; g_ucTmp++)
    {
        ga_ucLeftHallRxdBuf[g_ucTmp] = 0;
        ga_ucRightHallRxdBuf[g_ucTmp] = 0;
    }
    g_ucHallSavePst = 0;
    g_fLeftHallCurCnt = 0;
    g_fRightHallCurCnt = 0;

    //���ϱ�־��
    g_bForwardFlag = 0;
    g_bBackwardFlag = 0;
    g_bLeftwardFlag = 0;
    g_bRightwardFlag = 0;
    g_bNoObstacleFlag = 1;

    g_bSingleLostFlag = 0;
    g_bSoundStartFlag = 0;
    g_bSoundSpecialFlag = 0;
}
/*********************************/
/*-------- ����ʱ����־�� --------*/
/*********************************/
void GenerateTimeFlag()
{
    if(g_b1msFlag)
    {
        g_b1msFlag = 0;
        g_uiCnt++;
    }
    //����ʱ���־��
    if(g_uiCnt % T_Sampling == 0 & g_uiCnt != 0)
    {
        g_bSamplingFlag = 1;
    }
    if(g_uiCnt % T_Work == 0 & g_uiCnt != 0)
    {
        g_bWorkFlag = 1;
    }
    if(g_uiCnt % 10 == 0 & g_uiCnt != 0)
    {
        g_b10msFlag = 1;
    }

    if(g_uiCnt % 50 == 0 & g_uiCnt != 0)
    {
        g_b50msFlag = 1;
    }
    if(g_uiCnt % 100 == 0 & g_uiCnt != 0)
    {
        g_b100msFlag = 1;
    }
    if(g_uiCnt % 500 == 0 & g_uiCnt != 0)
    {
        g_b500msFlag = 1;
    }
    if(g_uiCnt % 1000 == 0 & g_uiCnt != 0)
    {
        g_b1sFlag = 1;
    }
    if(g_uiCnt % 3000 == 0 & g_uiCnt != 0)
    {
        g_b3sFlag = 1;
    }
    if(g_uiCnt % 5000 == 0 & g_uiCnt != 0)
    {
        g_b5sFlag = 1;
    }
    if(g_uiCnt == 6e4)
//		if(g_uiCnt % 3000 == 0 & g_uiCnt != 0)
    {
        g_b1minFlag = 1;
    }
    if(g_uiCnt > 6e4)
    {
        g_uiCnt = 0;
    }
    //�������ӱ�־��
    if(g_b1minFlag == 1)
    {
        g_uiMinCnt++;
    }
    if(g_uiMinCnt % 3 == 0 & g_uiMinCnt != 0)
    {
        g_b3minFlag = 1;
    }
    if(g_uiMinCnt > 250)
    {
        g_uiMinCnt = 0;
    }
}
/******************************************/
/*-------- ���ÿһ�ֲ�������ʱ���� --------*/
/******************************************/
void ClearMeaBuf()
{
    for(g_ucTmp = 0; g_ucTmp < 4; g_ucTmp++) //��վ���ֵ
    {
        ga_iDistance[g_ucTmp] = 0;
    }
    g_fLeftCurV = 0;
    g_fRightCurV = 0;
    g_fCurX = 0;
    g_fCurY = 0;
    g_fCurZ = 0;
    g_fCurR = 0;
    g_fCurA = 90;
    g_bForwardFlag = 0;
    g_bBackwardFlag = 0;
    g_bLeftwardFlag = 0;
    g_bRightwardFlag = 0;
}
/***********************************/
/*-------- ��������ֵ��� --------*/
/***********************************/
bit CheckDistance()
{
    int i_aiDstSorted[4], i_aucDstNum[4]; //ǰ�ߴӴ�С�������ֵ�����߶�Ӧ����ǰ�ߵ�ULS���������
    int i_iRangeFull4;
    unsigned char i_ucTmpA, i_ucTmpB, i_ucCnt;
    bit i_bDistanceOK;
    i_bDistanceOK = 0;

    for(i_ucTmpA = 0; i_ucTmpA < 4; i_ucTmpA++)//��Aλ�����б���
    {
        i_ucCnt = 0; //ÿ�ο�ʼʱ�Լ���������
        for(i_ucTmpB = 0; i_ucTmpB < 4; i_ucTmpB++) //��ʼ�������������м�������AС
        {
            if(i_ucTmpB == i_ucTmpA)
                continue;
            else
            {
                if(ga_iDistance[i_ucTmpA] > ga_iDistance[i_ucTmpB]) //���Aλ������Bλ�������������һ
                {
                    i_ucCnt++;
                }
            }
        }
        //i_ucCnt��ʾ�м�������AС��3-i_ucCnt�ɱ�ʾi_aiDstSorted�����е�Ԫ����ţ��Ӵ�С��
        i_aiDstSorted[3 - i_ucCnt] = ga_iDistance[i_ucTmpA];
        i_aucDstNum[3 - i_ucCnt] = i_ucTmpA;
    }

    //��������ɸѡ
    i_iRangeFull4 = i_aiDstSorted[0] - i_aiDstSorted[3]; //����4�����ݵļ���
    if(i_iRangeFull4 < DeltaDistance) //���������ݶ��ȽϾ�һ
    {
        i_bDistanceOK = 1;
    }
    return i_bDistanceOK;
}
/***************************************/
/*-------- ��̬����Ƕȷ������� --------*/
/***************************************/
void StaticAngleFDBK()
{
    float i_fTurningSpeed;

    if(g_fAngleCurError > StaticDeltaAngle & g_fAngleCurError < 50)
    {
        i_fTurningSpeed = UltraLowSpeed;
    }
    if(g_fAngleCurError > 50 & g_fAngleCurError < 120)
    {
        i_fTurningSpeed = LowSpeed;
    }
    if(g_fAngleCurError >  120)
    {
        i_fTurningSpeed = TurningSpeed;
    }

    g_fLeftSetV = 0; //Ԥ�����ٶȣ�ֻ���ض�������ٶ�
    g_fRightSetV = 0;

    if(fabs(g_fCurX) > MinX) //�ų�X�ĸ���
    {
        if(g_fAngleCurError > StaticDeltaAngle) //�����ǰ�ǶȺ�Ŀ��Ƕ�����DeltaAngle�����޷�Ӧ
        {
            g_fLeftSetV = abs(min(g_iSign, 0)) * i_fTurningSpeed;
            g_fRightSetV = max(g_iSign, 0) * i_fTurningSpeed;
        }
    }
}
/***************************************/
/*-------- ��̬����Ƕȷ������� --------*/
/***************************************/
void DynamicAngleFDBK()
{
    float i_fTurningSpeed;

    if(g_fAngleCurError > StaticDeltaAngle & g_fAngleCurError < 50)
    {
        i_fTurningSpeed = TurningSpeed;
    }
    if(g_fAngleCurError > 50 & g_fAngleCurError < 120)
    {
        i_fTurningSpeed = LowSpeed;
    }
    if(g_fAngleCurError >  120)
    {
        i_fTurningSpeed = UltraLowSpeed;
    }

    g_fLeftSetV = NormalSpeed; //Ԥ����ٶȣ�ֻ���ض�������ٶȲ�
    g_fRightSetV = NormalSpeed;

    if(fabs(g_fCurX) > MinX) //�ų�X�ĸ���
    {
        if(g_fCurR < 2 * MinR) //�����еȣ����߱�ת
        {
            if(g_fAngleCurError > 90) //����90�㣬��ͣ��ת
            {
                StaticAngleFDBK();
            }
            else if(g_fAngleCurError > DynamicDeltaAngle) //�����ǰ�ǶȺ�Ŀ��Ƕ�����DeltaAngle�����޷�Ӧ
            {
                if(g_iSign > 0) //����ڶ�����/��ߣ���������
                {
                    g_fLeftSetV = i_fTurningSpeed;
                }
                else if(g_iSign < 0) //������ڵ�һ����/�ұߣ��ҵ������
                {
                    g_fRightSetV = i_fTurningSpeed;
                }
            }
        }
        else if(g_fCurR > 2 * MinR) //�����Զ����ͣ��ת
        {
            g_fLeftSetV = 1.2*NormalSpeed; //Ԥ����ٶȣ�ֻ���ض�������ٶȲ�
            g_fRightSetV = 1.2*NormalSpeed;
            if(g_fAngleCurError > 90) //�����ǰ�ǶȺ�Ŀ��Ƕ�����DeltaAngle�����޷�Ӧ
            {
                StaticAngleFDBK(); //ת�뾲̬ת��ģʽ
            }
            else if(g_fAngleCurError > DynamicDeltaAngle) //�����ǰ�ǶȺ�Ŀ��Ƕ�����DeltaAngle�����޷�Ӧ
            {
                if(g_iSign > 0) //����ڶ�����/��ߣ���������
                {
                    g_fLeftSetV = 1.2 * i_fTurningSpeed;
                }
                else if(g_iSign < 0) //������ڵ�һ����/�ұߣ��ҵ������
                {
                    g_fRightSetV = 1.2 * i_fTurningSpeed;
                }
            }

//            if(g_fAngleCurError > DynamicDeltaAngle) //�����ǰ�ǶȺ�Ŀ��Ƕ�����DeltaAngle�����޷�Ӧ
//            {
//                StaticAngleFDBK(); //ת�뾲̬ת��ģʽ
//            }
//            else
//            {
//                g_fLeftSetV = FastSpeed; //Ԥ����ٶȣ�ֻ���ض�������ٶȲ�
//                g_fRightSetV = FastSpeed;
//            }
        }
    }
}
/*******************************/
/*-------- �����ϰ���� --------*/
/*******************************/
void CheckObstacle()
{
    if(P36 == 1)
    {
        g_bLeftwardFlag = 0;
    }
    else
    {
        g_bLeftwardFlag = 1;
    }
    if(P37 == 1)
    {
        g_bRightwardFlag = 0;
    }
    else
    {
        g_bRightwardFlag = 1;
    }

    if(g_bLeftwardFlag == 0 & g_bRightwardFlag == 0)
    {
        g_bNoObstacleFlag = 1;
    }
    else
    {
        g_bNoObstacleFlag = 0;
    }
}
/************************************/
/*-------- �����ٶȷ������� --------*/
/************************************/
void LeftVelocityFDBK(float i_fSetV)
{
    float k_p, k_i, k_d;
    float CurError, LastError, Last2Error;

    Last2Error = g_fLeftLastError;
    g_fLeftLastError = g_fLeftCurError;
    g_fLeftCurError = i_fSetV - g_fLeftCurV;

    LastError = g_fLeftLastError;
    CurError = g_fLeftCurError;

    if(i_fSetV == 0)
    {
        g_uiLeftCurDuty = 10;
        PWMLeftCon(g_uiLeftCurDuty);
    }
    else if(i_fSetV == ConstPWM)
    {
        g_uiLeftCurDuty = ConstDuty;
        PWMLeftCon(g_uiLeftCurDuty);
    }
    else if(i_fSetV == StopPWM)
    {
        g_uiLeftCurDuty = 10;
        PWMLeftCon(g_uiLeftCurDuty);
    }
    else
    {
        k_p = K_P;
        k_i = K_I;
        k_d = K_D;
        g_uiLeftCurDuty += k_p * (CurError - LastError) + k_i * (CurError + LastError) / 2 + k_d * (CurError - 2 * LastError + Last2Error);
        PWMLeftCon(g_uiLeftCurDuty);
    }
}
/************************************/
/*-------- �ҵ���ٶȷ������� --------*/
/************************************/
void RightVelocityFDBK(float i_fSetV)
{
    float k_p, k_i, k_d;
    float CurError, LastError, Last2Error;

    Last2Error = g_fRightLastError;
    g_fRightLastError = g_fRightCurError;
    g_fRightCurError = i_fSetV - g_fRightCurV;

    LastError = g_fRightLastError;
    CurError = g_fRightCurError;

    if(i_fSetV == 0)
    {
        g_uiRightCurDuty = 10;
        PWMRightCon(g_uiRightCurDuty);
    }
    else if(i_fSetV == ConstPWM)
    {
        g_uiRightCurDuty = ConstDuty;
        PWMRightCon(g_uiRightCurDuty);
    }
    else if(i_fSetV == StopPWM)
    {
        g_uiRightCurDuty = 10;
        PWMRightCon(g_uiLeftCurDuty);
    }
    else
    {
        k_p = K_P;
        k_i = K_I;
        k_d = K_D;
        g_uiRightCurDuty += k_p * (CurError - LastError) + k_i * (CurError + LastError) / 2 + k_d * (CurError - 2 * LastError + Last2Error);
        PWMRightCon(g_uiRightCurDuty);
    }
}
/*********************************/
/*-------- ���㷢������� --------*/
/*********************************/
void CalLocation()
{
    float xdata i_fX1, i_fX2, i_fY1, i_fY2;

    //��ʼ���㣬�µ������������
    i_fY1 = (pow(ga_iDistance[0], 2) - pow(ga_iDistance[1], 2)) / 2.0 / DIS_S;
    i_fX1 = (pow(ga_iDistance[1], 2) - pow(ga_iDistance[2], 2)) / 2.0 / DIS_S;
    i_fY2 = (pow(ga_iDistance[3], 2) - pow(ga_iDistance[2], 2)) / 2.0 / DIS_S;
    i_fX2 = (pow(ga_iDistance[0], 2) - pow(ga_iDistance[3], 2)) / 2.0 / DIS_S;

    //�����ݽ��д���
    g_fCurX = (i_fX1 + i_fX2) / 2.0;
    g_fCurY = (i_fY1 + i_fY2) / 2.0;
    g_fCurR = pow(g_fCurX * g_fCurX + g_fCurY * g_fCurY, 0.5);
    g_fCurA = atan2(g_fCurY, g_fCurX) * 180 / 3.14; //�Ƕ�ֵ
}
/*********************************/
/*-------- �洢���������ֵ --------*/
/*********************************/
void StoreLocation()
{
    ga_fCurXBuf[g_ucLocationSavePst] = g_fCurX;
    ga_fCurYBuf[g_ucLocationSavePst] = g_fCurY;
    g_ucLocationSavePst = (g_ucLocationSavePst + 1) & (LocationBufMaxBytes - 1); //��λ����

    g_fAngleLast2Error = g_fAngleLastError;
    g_fAngleLastError = g_fAngleCurError;
    if(g_fCurX < 0)
    {
        g_iSign = 1; //����
        g_fAngleCurError = (g_fCurA - g_fSetA) * g_iSign;
        if(g_fCurY < 0)
        {
            g_fAngleCurError = 180 + g_fCurA + 90; //��Ե������޵ķ����λ�ã��ԽǶȽ�������
        }
    }
    else
    {
        g_iSign = -1; //����
        g_fAngleCurError = (g_fCurA - g_fSetA) * g_iSign;
    }
}

/*********************************/
/*-------- ��ս��������� --------*/
/*********************************/
void ClearRxdBuf()
{
    for(g_ucTmp = 0; g_ucTmp < UARTBufMaxBytes; g_ucTmp++)
    {
        ga_ucUartRxdBuf[g_ucTmp] = 0;
    }
    g_ucUART2SavePst = 0;
}
/***********************************/
/*-------- ��ʱ����(ms����) --------*/
/***********************************/
void DelayTime(int n)
{
    int this_Cnt, this_tmp;
    for(this_Cnt = 0; this_Cnt < n; this_Cnt++)
    {
        for(this_tmp = 0; this_tmp < 1350; this_tmp++)
        {
        }
    }
}
/******************************************/
/*-------- �������������ݵ������˲� --------*/
/******************************************/
void HallCntFiltering()
{
    unsigned char i_ucLeftNonZeroNum, i_ucRightNonZeroNum,  i_ucLeftBiggerCnt, i_ucRightBiggerCnt, i_ucTmpA, i_ucTmpB;
    unsigned char xdata ia_ucLeftHallSorted[HallBufMaxBytes], ia_ucRightHallSorted[HallBufMaxBytes]; //����������
    int i_iLeftSum, i_iRightSum; //�������

    //�������������ݵĴ��
    ga_ucLeftHallRxdBuf[g_ucHallSavePst] = g_ucLeftHallCnt;
    ga_ucRightHallRxdBuf[g_ucHallSavePst] = g_ucRightHallCnt;
    g_ucHallSavePst = (g_ucHallSavePst + 1) & (HallBufMaxBytes - 1); //��λ���Σ���ֹ���

    //������ʼ��
    g_fLeftHallCurCnt = g_ucLeftHallCnt; //�˲�ǰ���г�ʼ������ֹ���ݳ���
    g_fRightHallCurCnt = g_ucRightHallCnt;
    i_ucLeftBiggerCnt = 0;
    i_ucRightBiggerCnt = 0;
    i_ucLeftNonZeroNum = 0; //��0Ԫ�ظ���
    i_ucRightNonZeroNum = 0; //��0Ԫ�ظ���

    //�ײ��������
    g_ucLeftHallCnt = 0;
    g_ucRightHallCnt = 0;

    //�Դ�ŵ����ݽ��д���
    for(i_ucTmpA = 0; i_ucTmpA < HallBufMaxBytes; i_ucTmpA++)
    {
        if(ga_ucLeftHallRxdBuf[i_ucTmpA] != 0)
        {
            i_ucLeftNonZeroNum++;
        }
        if(ga_ucRightHallRxdBuf[i_ucTmpA] != 0)
        {
            i_ucRightNonZeroNum++;
        }
        ia_ucLeftHallSorted[i_ucTmpA] = 0; //��ʼ����������
        ia_ucRightHallSorted[i_ucTmpA] = 0; //��ʼ����������
    }

    //�������Ԫ�ض��Ƿ������飬����Կ�ʼ�˲��ˣ���������һ���ڵļ���ֵ��Ϊ���ս��
    if(i_ucLeftNonZeroNum == HallBufMaxBytes & i_ucRightNonZeroNum == HallBufMaxBytes)
    {
        i_ucLeftBiggerCnt = 0; //ÿ�α���ǰ�������㣡
        i_ucRightBiggerCnt = 0;
        //����С��������
        for(i_ucTmpA = 0; i_ucTmpA < HallBufMaxBytes; i_ucTmpA++)
        {
            //��������HALL����
            for(i_ucTmpB = 0; i_ucTmpB < HallBufMaxBytes; i_ucTmpB++)
            {
                //���AλԪ�غ�BλԪ�ض��Ƿ���Ԫ�أ�����бȽ�
                if(ga_ucLeftHallRxdBuf[i_ucTmpA] > ga_ucLeftHallRxdBuf[i_ucTmpB])
                {
                    i_ucLeftBiggerCnt++; //��A��Ԫ�ر�B��Ԫ�ش󣬼�����+1
                }
            }
            //������ϣ���ʼ�������A��Ԫ�ش������������飬��A��Ԫ���ڷ���Ԫ���н������򣬵�0λ�����ġ�
            ia_ucLeftHallSorted[i_ucLeftNonZeroNum - 1 - i_ucLeftBiggerCnt] = ga_ucLeftHallRxdBuf[i_ucTmpA];

            //�����ҵ��HALL����
            for(i_ucTmpB = 0; i_ucTmpB < HallBufMaxBytes; i_ucTmpB++)
            {
                //���AλԪ�غ�BλԪ�ض��Ƿ���Ԫ�أ�����бȽ�
                if(ga_ucRightHallRxdBuf[i_ucTmpA] > ga_ucRightHallRxdBuf[i_ucTmpB])
                {
                    i_ucRightBiggerCnt++; //��A��Ԫ�ر�B��Ԫ�ش󣬼�����+1
                }
            }
            //������ϣ���ʼ�������A��Ԫ�ش������������飬��A��Ԫ���ڷ���Ԫ���н������򣬵�0λ�����ġ�
            ia_ucRightHallSorted[i_ucRightNonZeroNum - 1 - i_ucRightBiggerCnt] = ga_ucRightHallRxdBuf[i_ucTmpA];
        }

        //���ݽ�����˲����������ʱ����g_fLeftHallCurCnt��g_fRightHallCurCnt
        i_iLeftSum = 0;
        i_iRightSum = 0;
        for(i_ucTmpA = 1; i_ucTmpA < HallBufMaxBytes - 1; i_ucTmpA++) //�ӵ�2�������ʼͳ�ƣ�ֱ����2С�������ܿ�������С��
        {
            i_iLeftSum += ia_ucLeftHallSorted[i_ucTmpA];
            i_iRightSum += ia_ucRightHallSorted[i_ucTmpA];
        }
        g_fLeftHallCurCnt = 1.0 * i_iLeftSum / (HallBufMaxBytes - 2); //����˲����
        g_fRightHallCurCnt = 1.0 * i_iRightSum / (HallBufMaxBytes - 2);
    }
}
/************************************/
/*-------- �����LED�ƵĿ��� --------*/
/************************************/
void LedToogle(int n)
{
    //��˸һ��
    P20 = 0;
    P21 = 0;
    P22 = 0;
    P23 = 0;
    P24 = 0;
    P25 = 0;
    if (n == 0)
    {
        P20 = 1;
    }
    else if(n == 1)
    {
        P21 = 1;
    }
    else if(n == 2)
    {
        P22 = 1;
    }
    else if(n == 3)
    {
        P23 = 1;
    }
    else if(n == 4)
    {
        P24 = 1;
    }
    else if(n == 5)
    {
        P25 = 1;
    }
    DelayTime(1);
    P20 = 0;
    P21 = 0;
    P22 = 0;
    P23 = 0;
    P24 = 0;
    P25 = 0;
}

/********************************/
/*-------- PWMLeft�Ŀ��� --------*/
/********************************/
void PWMLeftCon(unsigned int duty)
//��ǿ��PWM0(�������������P17
{
    unsigned int i_iHighStart;
    unsigned char i_ucPCA;
    if(duty > 1024)
    {
        duty = 1024;
        g_uiLeftCurDuty = duty;
    }
    if(duty < 10)
    {
        duty = 10;
        g_uiLeftCurDuty = duty;
    }
    i_iHighStart = 1024 - g_uiLeftCurDuty;

    i_ucPCA = i_iHighStart >> 8;
    i_ucPCA = (i_ucPCA << 2) | (i_ucPCA << 4);

    CR = 0;
    CCON = 0x00;
    CMOD = 0x04; //PCA ʱ��ΪT0�������
    CL = 0x00;
    CH = 0x00;

    CCAPM0 = 0x42; //PCA ģ�� 0 Ϊ PWM ����ģʽ
    PCA_PWM0 = 0xc0;//PCA ģ�� 0 ��� 10 λ PWM
    //д��װ��ֵ���Ƚ�ֵ
    PCA_PWM0 |= i_ucPCA; //д���λ����ֵ���λ�Ƚ�ֵ
    CCAP0H = i_iHighStart; //��λ����ֵ
    CCAP0L = CCAP0H; //��λ�Ƚ�ֵ

    CR = 1; //���� PCA ��ʱ��
}
/********************************/
/*-------- PWMRightCon���� --------*/
/********************************/
void PWMRightCon(unsigned int duty)
//��ǿ��PWM1�������P16
{
    unsigned int i_iHighStart;
    unsigned char i_ucPCA;
    if(duty > 1024)
    {
        duty = 1024;
        g_uiRightCurDuty = duty;
    }
    if(duty < 10)
    {
        duty = 10;
        g_uiRightCurDuty = duty;
    }
    i_iHighStart = 1024 - g_uiRightCurDuty;

    i_ucPCA = i_iHighStart >> 8;
    i_ucPCA = (i_ucPCA << 2) | (i_ucPCA << 4);

    CR = 0;
    CCON = 0x00;
    CMOD = 0x04; //PCA ʱ��ΪT0�������
    CL = 0x00;
    CH = 0x00;

    CCAPM1 = 0x42; //PCA ģ�� 1 Ϊ PWM ����ģʽ
    PCA_PWM1 = 0xc0;//PCA ģ�� 1 ��� 10 λ PWM
    //д��װ��ֵ���Ƚ�ֵ
    PCA_PWM1 |= i_ucPCA; //д���λ����ֵ���λ�Ƚ�ֵ
    CCAP1H = i_iHighStart; //��λ����ֵ
    CCAP1L = CCAP1H; //��λ�Ƚ�ֵ

    CR = 1; //���� PCA ��ʱ��
}
/*********************************/
/*-------- �����ܵ����� --------*/
/*********************************/
unsigned char CheckUartRxd() //����Ƿ���ULS���ն˷��ص�����
{
    unsigned char k, ucData_OK;
    ucData_OK = 0;
    k = 0;
    if (ga_ucUartRxdBuf[0] == 0x55) //֡ͷ1
    {
        k++;
    }
    if (ga_ucUartRxdBuf[1] == 0xAA) //֡ͷ2
    {
        k++;
    }
    if (ga_ucUartRxdBuf[2] == 0x56) //��λ����STC8A���ĵ�ַ
    {
        k++;
    }
    if (k == 3)
    {
        ucData_OK = GeneralULSFrame; //��ULS�����ݣ�����֪����R�˻���T��
        if (ga_ucUartRxdBuf[4] == 0x02 && ga_ucUartRxdBuf[5] == 0xD1)
            //T����Ӧ����
        {
            ucData_OK = TxdULSFrame;
        }
        if (ga_ucUartRxdBuf[4] == 0x0D)
            //R����Ӧ����
        {
            ucData_OK = RxdULSFrame;
        }
    }
    return (ucData_OK);
}
/******************************/
/*-------- Ӳ����ʼ�� --------*/
/*****************************/
void HardwareInit()
{
    PIN_LGnd = 0;
    PIN_RGnd = 0;

    //�������ģʽ�����ڵ���������ϵ�LED��
    P2M0 = 0xff;
    P2M1 = 0x00;

    //�ⲿ�ж�INT0��ʼ����P32
    IT0 = 1; //INT0 �½��ش���
    EX0 = 1; //ʹ��INT0 �ж�

    //�ⲿ�ж�INT1��ʼ����P33
    IT1 = 1; //INT1 �½��ش���
    EX1 = 1; //ʹ��INT1 �ж�

//    //�ⲿ�ж�INT2��ʼ����P36
//    INTCLKO = EX2; //ʹ��INT2 �½����ж�

//    //�ⲿ�ж�INT3��ʼ����P37
//    INTCLKO = EX3; //ʹ��INT3 �½����ж�

    //��ʱ��0����PWM���ڵ�ѡ��
    AUXR |= AUXR_T0x1; //T0���ڲ���Ƶģʽ
    TMOD |= Timer0_M0; //T0����ģʽ0��16λ�Զ�����
    TL0 = BRT0;
    TH0 = BRT0 >> 8;
    TF0 = 0;
    ET0 = 1; //T0�ж�ʹ��
    TR0 = 1; //T1��ʼ����

    //��ʱ��1��Ϊ1msʱ���жϵĳ�ʼ��
    //�ص㣺T1ģʽ��ѡ���Ƿ��Ƶ����ʼ�ߵ�λ��
    AUXR |= AUXR_T1x12; //��ʱ��1������12��Ƶ
    TMOD |= Timer1_M0; //T1����ģʽ1��16λ�Զ�����
    TL1 = 0x66;
    TH1 = 0xFC; //����T1���Զ�����ֵ
    ET1 = 1; //����ʱ��1�����ж�
    TR1 = 1; //����ʱ��1��ʼ����

    //��ʱ��2���ڴ���2�Ĳ����ʷ���������Uart2Init()�г�ʼ��
    //��ʱ��3���ڴ���1�Ĳ����ʷ���������Uart3Init()�г�ʼ��
}
/*********************************/
/*-------- ����2�ĳ�ʼ�� --------*/
/*********************************/
void Uart2Init()
{
    //���ô���2
    S2CON = UART2_M0; //����2����ģʽ0���ɱ䲨����8λ���ݣ��ö�ʱ��2�������ʷ�����
    S2CON |= S2REN; //����2��������

    //���ö�ʱ��2
    AUXR |= AUXR_T2x12; // ��ʱ��2����12��Ƶ
    //T2�̶�Ϊ16λ����ģʽ;
    TL2 = BRT2;
    TH2 = BRT2 >> 8; // ���ö�ʱ��2�ĵ�/�ߵ�ַ��ֵ��ȷ��������2

    //��ʱ��ʹ��
    AUXR |= T2R; //����ʱ��2��ʼ����

    //�ж�ʹ��
    IE2 |= ES2; //������2���ж�
}
/*********************************/
/*-------- ����3�ĳ�ʼ�� --------*/
/*********************************/
void Uart3Init()
{
    //���ô���1
    S3CON = UART3_M0T3; //����3����ģʽ0���ɱ䲨����8λ���ݣ��ö�ʱ��3�������ʷ���������������

    //���ö�ʱ��3
    T3L = BRT3;
    T3H = BRT3 >> 8;

    //��ʱ��ʹ��
    T4T3M = 0x0a; //1TƵ�ʣ�����Ƶ����ʹ��T3

    //�ж�ʹ��
    IE2 |= ES3; //������3���ж�
}
/**************************************/
/*-------- ����2����BYTE������ --------*/
/**************************************/
void SendByUart2(unsigned char dat)
{
    while(g_bUart2BusyFlag); //���ֿ�ת���ȴ�g_bUart3BusyFlag=0���༴�ȴ����ڵ�TI�ж�
    g_bUart2BusyFlag = 1; //����ǰ����g_bUart3BusyFlag��1
    S2BUF = dat; //�������ݣ�������ɺ�g_bUart3BusyFlag�ᱻ��0
}
/**************************************/
/*-------- ����3����BYTE������ --------*/
/**************************************/
void SendByUart3(unsigned char dat)
{
    while(g_bUart3BusyFlag); //���ֿ�ת���ȴ�g_bUart3BusyFlag=0���༴�ȴ����ڵ�TI�ж�
    g_bUart3BusyFlag = 1; //����ǰ����g_bUart3BusyFlag��1
    S3BUF = dat; //�������ݣ�������ɺ�g_bUart3BusyFlag�ᱻ��0
}
/****************************/
/*-------- �������� --------*/
/****************************/
void TxdData(int mode)
{
    unsigned char i_ucLen;
    unsigned char i_uctmp;
    if(mode == SoundParameter)
    {
        char i_caTmp[20];
        char i_caTmp1[20];
//			  char xdata i_caText[50] = {"[v1][s8][m52]��ռ�ձ�"};
//        char i_caText1[20] = {"��ռ�ձ�"};
//        sprintf(i_caTmp, "%3.1f", g_uiLeftCurDuty);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%3.1f", g_uiRightCurDuty);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

//        char xdata i_caText[50] = {"[v1][s8][m52]���ٶ�"};
//        char i_caText1[20] = {"���ٶ�"};
//        sprintf(i_caTmp, "%0.3f", g_fLeftCurV);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%0.3f", g_fRightCurV);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

//        char xdata i_caText[50] = {"[v1][s9][m52]�����"};
//        char xdata i_caText1[20] = {"�һ���"};
//        sprintf(i_caTmp, "%2.1f", g_fLeftHallCurCnt);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%2.1f", g_fRightHallCurCnt);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

//        char i_caText[40] = {"[v1][s8][m52]X����"};
//        char i_caText1[20] = {"Y����"};
//        sprintf(i_caTmp, "%4.0f", g_fCurX);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%4.0f", g_fCurY);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);

        char i_caText[40] = {"[v1][s8][m52]����"};
        char i_caText1[20] = {"�Ƕ�"};
        sprintf(i_caTmp, "%4.0f", g_fCurR);
        strcat(i_caText, i_caTmp);
        sprintf(i_caTmp1, "%3.1f", g_fCurA);
        strcat(i_caText1, i_caTmp1);
        strcat(i_caText, i_caText1);

//        char xdata i_caText[50] = {"[v1][s9][m52]��ͨ��"};
//        char xdata i_caText1[20] = {"��һͨ��"};
//				char xdata i_caText2[20] = {"����ͨ��"};
//				char xdata i_caText3[20] = {"����ͨ��"};
//        sprintf(i_caTmp, "%u", ga_iDistance[0]);
//        strcat(i_caText, i_caTmp);
//        sprintf(i_caTmp1, "%u", ga_iDistance[1]);
//        strcat(i_caText1, i_caTmp1);
//        strcat(i_caText, i_caText1);
//				sprintf(i_caTmp, "%u", ga_iDistance[2]);
//        strcat(i_caText2, i_caTmp);
//        sprintf(i_caTmp1, "%u", ga_iDistance[3]);
//        strcat(i_caText3, i_caTmp1);
//        strcat(i_caText2, i_caText3);
//				strcat(i_caText, i_caText2);

        i_ucLen = strlen(i_caText);
        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else if (mode == SoundSignalLost) //����ģ�鷢��
    {
        char i_caText[] = {"[v3][s9][m51]�����źŶ�ʧ��"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }

    else  if (mode == SoundStart) //����
    {
        char i_caText[] = {"[v3][s9][m3]��ӭʹ��B39չλ��С����ϲ�����B39ͶƱ"};
//        char xdata i_caText[] = {"[v2][s9][m3]�����׿����ܸ���С��������������С������ƶ��������˷��죬������һ���ĸ������飡"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundTooFar) //����ģ�鷢��
    {
        char i_caText[] = {"[v3][s9][m52]̫Զ�ˣ���ȵ��ҡ�"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundObstacle) //����ģ�鷢��
    {
        char xdata i_caText[] = {"[v3][s9][m3]ǰ�����ϰ�"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundSpecial) //����ģ�鷢��
    {
        char xdata i_caText[120] = {"[v3][s9][m3]�����׿����ܸ���С��������������С������ƶ��������˷��죬������һ���ĸ������飡"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundSignalGot) //����ģ�鷢��
    {
        char i_caText[] = {"[v2][s9][m52]�յ��ź�"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundLeft) //����ģ�鷢��
    {
        char i_caText[] = {"[v3][s10][m52]��"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundRight) //����ģ�鷢��
    {
        char i_caText[] = {"[v3][s10][m52]��"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else  if (mode == SoundStraight) //����ģ�鷢��
    {
        char i_caText[] = {"[v3][s10][m52]ֱ"};
        i_ucLen = strlen(i_caText);

        ga_ucUartTxdBuf[0] = 0xFD; //֡ͷ
        ga_ucUartTxdBuf[1] = 0x00; //���ȸ��ֽ�
        ga_ucUartTxdBuf[2] = i_ucLen + 2; //���ȵ��ֽ�
        ga_ucUartTxdBuf[3] = 0x01; //������
        ga_ucUartTxdBuf[4] = 0x00; //�����ʽ
        i_ucLen = strlen(ga_ucUartTxdBuf);
        for(i_uctmp = 0; i_uctmp < 5; i_uctmp++)
        {
            SendByUart3(ga_ucUartTxdBuf[i_uctmp]);
        }

        i_ucLen = strlen(i_caText);
        for(i_uctmp = 0; i_uctmp < i_ucLen; i_uctmp++)
        {
            SendByUart3(i_caText[i_uctmp]);
        }
    }
    else
    {
        ga_ucUartTxdBuf[0] = 0x55; //֡ͷ1
        ga_ucUartTxdBuf[1] = 0xAA; //֡ͷ2
        ga_ucUartTxdBuf[3] = 0x56; //STC8A�ĵ�ַ��Ϊ�̶�ֵ

        if (mode == FindDistance) //����ն˲��Ҿ���
        {
            ga_ucUartTxdBuf[2] = 0x10; //
            ga_ucUartTxdBuf[4] = 0x02; //
            ga_ucUartTxdBuf[5] = 0x63; //
            ga_ucUartTxdBuf[6] = 0x01; //
            ga_ucUartTxdBuf[7] = 0x9B; //
            for(i_uctmp = 0; i_uctmp < 8; i_uctmp++)
            {
                SendByUart2(ga_ucUartTxdBuf[i_uctmp]);
            }
        }
        else if (mode == FindTemperature) //����ն˲����¶�
        {
            ga_ucUartTxdBuf[2] = 0x10;
            ga_ucUartTxdBuf[3] = 0x56;
            ga_ucUartTxdBuf[4] = 0x02;
            ga_ucUartTxdBuf[5] = 0x52;
            ga_ucUartTxdBuf[6] = 0x01;
            ga_ucUartTxdBuf[7] = 0xAC;
            for(i_uctmp = 0; i_uctmp < 8; i_uctmp++)
            {
                SendByUart2(ga_ucUartTxdBuf[i_uctmp]);
            }
        }
        else if (mode == StartULS) //�������������������
        {
            ga_ucUartTxdBuf[2] = 0xF1;
            ga_ucUartTxdBuf[3] = 0x56;
            ga_ucUartTxdBuf[4] = 0x02;
            ga_ucUartTxdBuf[5] = 0x51;
            ga_ucUartTxdBuf[6] = 0x20;
            ga_ucUartTxdBuf[7] = 0x8E;
            for(i_uctmp = 0; i_uctmp < 8; i_uctmp++)
            {
                SendByUart2(ga_ucUartTxdBuf[i_uctmp]);
            }
        }
    }
}
/*-------- �жϴ��� --------*/
/*********************************************/
/*-------- �ⲿ�ж�0����������A�ࣩ --------*/
/*********************************************/
void External0_Int(void) interrupt 0 using 1
//�ⲿ�ж�0 INT0��ӦP3.2��
{
    //��λ���Σ���ֹ���
    g_ucLeftHallCnt = min(g_ucLeftHallCnt + 1, 255);
}
/*********************************************/
/*-------- �ⲿ�ж�1���ҵ������A�ࣩ --------*/
/*********************************************/
void External1_Int(void) interrupt 2 using 1
//�ⲿ�ж�1 INT1��ӦP3.3��
{
    //��λ���Σ���ֹ���
    g_ucRightHallCnt = min(g_ucRightHallCnt + 1, 255);
}
///************************************************/
///*-------- �ⲿ�ж�2�����ϽǺ��⴫������ --------*/
///***********************************************/
//void External2_Int(void) interrupt 10 using 1
////�ⲿ�ж�2 INT2��ӦP3.6��
//{
//    g_bLeftwardFlag = 1;
//}
///************************************************/
///*-------- �ⲿ�ж�3�����ϽǺ��⴫������ --------*/
///***********************************************/
//void External3_Int(void) interrupt 11 using 1
////�ⲿ�ж�2 INT2��ӦP3.6��
//{
//    g_bRightwardFlag = 1;
//}
/*********************************************/
/*-------- ��ʱ��0ʱ���жϣ�PWM���ڣ� --------*/
/*********************************************/
void Timer0_Int(void) interrupt 1 using 2
{

}
/*********************************************/
/*-------- ��ʱ��1ʱ���жϣ�1msһ�Σ� --------*/
/*********************************************/
void Timer1_Int(void) interrupt 3 using 2
{
    g_b1msFlag = 1;
}
/*********************************************/
/*-------- ����3���жϣ��շ����� --------*/
/*********************************************/
void UART3_Int(void) interrupt 17 using 1
{
    if(S3CON & S3RI)
    {
        S3CON &= ~S3RI;
    }
    if(S3CON & S3TI)
    {
        S3CON &= ~S3TI;
        g_bUart3BusyFlag = 0;
    }
}
/*********************************************/
/*-------- ����2���жϣ��շ����� --------*/
/*********************************************/
void UART2_Int(void) interrupt 8 using 1
{
    if(S2CON & S2RI)
    {
        S2CON &= ~S2RI;
        ga_ucUartRxdBuf[g_ucUART2SavePst] = S2BUF;
        g_ucUART2SavePst	= (g_ucUART2SavePst + 1) & (UARTBufMaxBytes - 1); // �������θ�λ�ķ�ʽʵ��ָ��Ļ��δ���
    }
    if(S2CON & S2TI)
    {
        S2CON &= ~S2TI;
        g_bUart2BusyFlag = 0;
    }
}
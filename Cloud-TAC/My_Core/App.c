/*********************************************************************************************************/
/*
                   _ooOoo_
                  o8888888o
                  88" . "88
                  (| -_- |)
                  O\  =  /O
               ____/`---'\____
             .'  \\|     |//  `.
            /  \\|||  :  |||//  \
           /  _||||| -:- |||||-  \
           |   | \\\  -  /// |   |
           | \_|  ''\---/''  |   |
           \  .-\__  `-`  ___/-. /
         ___`. .'  /--.--\  `. . __
      ."" '<  `.___\_<|>_/___.'  >'"".
     | | :  `- \`.;`\ _ /`;.`/ - ` : | |
     \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
                   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
         ????C??      ????BUG
*/
/*********************************************************************************************************/
#include "App.h"
/********************************************************************************************************
Function Name: System_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/

pid_t Motor1_Pid;  //???PID

pid_t YS_Motor1_Pid;  //???PID

extern DMA_HandleTypeDef hdma_tim3_ch1;
#define R04_MOTOR_COUNT 12U
#define R04_DEG2RAD(x) ((x) * 3.14159265f / 180.0f)
#define R04_RAD2DEG(x) ((x) * 180.0f / 3.14159265f)
#define R04_ARRAY_SIZE(arr) ((uint8_t)(sizeof(arr) / sizeof((arr)[0])))
#define R04_CMD_DEADBAND 0.20f
/* 前进：仅「一侧三足腾空、对侧三足撑地」时提高撑地腿 MIT 刚度/阻尼，减轻机身下沉；六足同时承力步不加（避免回弹上顶） */
#define R04_FWD_TRIPOD_STANCE_KP_MUL 1.40f
#define R04_FWD_TRIPOD_STANCE_KD_MUL 1.18f
/* 摇杆回中后等待多久才允许步态；0=使能后立即响应（仅 Sbus[4] 与回连有效时仍需回中） */
#define R04_REMOTE_ARM_MS 0U
#define R04_CONTROL_DISABLE_TH (-0.5f)
#define R04_CONTROL_ENABLE_TH  (0.5f)
/* Sbus[4] 使能后偶数轴相对使能前再“下压”撑起机身（度），符号可按实机反向改 */
/* 使能瞬间偶数轴相对“趴地”姿态多压下的角度；越大机身抬得越高，过小撑不起来，可实机微调 */
#define R04_BODY_LIFT_EVEN_DEG (26.0f)
/* 失能后仅偶数轴缓降回使能前角度，时长 ms（与 1ms 任务一致）；过短易砸地，过长体感拖沓 */
#define R04_EVEN_RAMP_DOWN_MS  (6000U)
/* 使能后偶数轴零位插值到撑起目标；过短易“砸地”，过长响应慢 */
#define R04_EVEN_ENABLE_RAMP_MS (4000U)
/* 插值结束后仍略软站立若干 ms 再切常规站立；已加强软站立增益，可略缩短 */
#define R04_POST_ENABLE_SOFT_MS (600U)
/* 1：仅前进爬行时对 m11(电机11、腿6水平) 表内角取反；站立/使能表为全 0 不改变。若反的是上下请改 0 并对 m12 另做 */
#define R04_M11_FORWARD_FB_INVERT 1

static const uint8_t R04_MotorIds[R04_MOTOR_COUNT] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

typedef enum
{
	R04_ACTION_STAND = 0,
	R04_ACTION_FORWARD_CRAWL,
	R04_ACTION_BACKWARD_CRAWL,
	R04_ACTION_STRAFE_LEFT,
	R04_ACTION_STRAFE_RIGHT,
	R04_ACTION_TURN_LEFT,
	R04_ACTION_TURN_RIGHT
} R04_ActionId;

typedef struct
{
	// Position order matches R04_MotorIds: {1,2,3,4,5,6,7,8,9,10,11,12}
	float pos_deg[R04_MOTOR_COUNT];
	uint16_t hold_ms;
	float speed;
	float torque;
	float kp;
	float kd;
} R04_ActionStep;

typedef struct
{
	const R04_ActionStep *steps;
	uint8_t step_count;
	uint8_t loop;
} R04_ActionGroup;

typedef struct
{
	R04_ActionId action_id;
	uint8_t step_idx;
	uint16_t step_tick;
} R04_ActionRuntime;

#define R04_STEP(hold_ms, speed, torque, kp, kd, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12) \
	{{(m1), (m2), (m3), (m4), (m5), (m6), (m7), (m8), (m9), (m10), (m11), (m12)}, (hold_ms), (speed), (torque), (kp), (kd)}

static float R04_Abs(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static uint8_t R04_IsRemoteNeutral(void)
{
	return ((R04_Abs(Sbus_Value[2]) < R04_CMD_DEADBAND) &&
	        (R04_Abs(Sbus_Value[0]) < R04_CMD_DEADBAND) &&
	        (R04_Abs(Sbus_Value[3]) < R04_CMD_DEADBAND)) ? 1U : 0U;
}

static const R04_ActionStep R04_StandSteps[] =
{
	/* 站立：略提高 kp 撑自重 */
	R04_STEP(300U, 0.58f, 0.0f, 30.0f, 0.57f,
	         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
};

/* 使能插值/缓冲：接近常规站立，避免启动撑不住 */
static const R04_ActionStep R04_StandStepsEnableSoft =
	R04_STEP(300U, 0.56f, 0.0f, 28.0f, 0.56f,
	         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

static const R04_ActionStep R04_ForwardCrawlSteps[] =
{
	/*  m1     m2     m3     m4     m5     m6     m7     m8     m9    m10    m11    m12  */
	/* 每腿两轴：奇数=前后(fb)，偶数=上下(ud) */
	/*
	 * 腿号：1 左上(m1-2) 2 左中(m3-4) 3 左下(m5-6) 4 右下(m7-8) 5 右中(m9-10) 6 右上(m11-12)
	 * 三足 A=腿1、3、5   三足 B=腿2、4、6
	 * 共 10 步 loop：
	 *   ①②a②b：A 抬→前摆→落，B 不动
	 *   ③a③b：六条腿水平轴一起后蹬（机体前移）；末态 A 水平≈中位(0)，B 水平≈后侧
	 *   ④⑤a⑤b：B 抬→前摆→落，A 保持③b
	 *   ⑥a⑥b：六条腿一起后蹬；末态 B 水平≈中位，A 水平≈后侧 → 回①
	 */

	/* ① 三足 A(1、3、5) 抬起；B(2、4、6) 不动 */
	R04_STEP(240U, 0.65f, 0.0f, 44.0f, 0.58f,
	        -18.0f, 30.0f,  0.0f,  0.0f,-18.0f, 30.0f,
	          0.0f,  0.0f, 18.0f,-30.0f,  0.0f,  0.0f),
	/* ②a A 完全抬起后向前摆；B 不动 */
	R04_STEP(624U, 0.0f, 0.0f, 24.0f, 0.72f,
	         22.0f, 30.0f,  0.0f,  0.0f, 22.0f, 30.0f,
	          0.0f,  0.0f,-22.0f,-30.0f,  0.0f,  0.0f),
	/* ②b A 放下 */
	R04_STEP(496U, 0.30f, 0.0f, 18.0f, 0.64f,
	         22.0f,  0.0f,  0.0f,  0.0f, 22.0f,  0.0f,
	          0.0f,  0.0f,-22.0f,  0.0f,  0.0f,  0.0f),
	/* ③a 全体后蹬前半：六腿水平(fb)一起从 ②b 向「A 中、B 后」插值 */
	R04_STEP(160U, 0.62f, 0.0f, 42.0f, 0.58f,
	         11.0f,  0.0f,-11.0f,  0.0f, 11.0f,  0.0f,
	         11.0f,  0.0f,-11.0f,  0.0f, -11.0f,  0.0f),
	/* ③b 全体后蹬末：腿1、3、5 水平≈中位；腿2、4、6 水平≈后侧（机体已前移） */
	R04_STEP(160U, 0.62f, 0.0f, 42.0f, 0.58f,
	          0.0f,  0.0f,-22.0f,  0.0f,  0.0f,  0.0f,
	         22.0f,  0.0f,  0.0f,  0.0f, -22.0f,  0.0f),
	/* ④ B 抬起；A 保持 ③b（水平全 0） */
	R04_STEP(240U, 0.65f, 0.0f, 44.0f, 0.58f,
	          0.0f,  0.0f,-22.0f, 30.0f,  0.0f,  0.0f,
	         22.0f,-30.0f,  0.0f,  0.0f, 22.0f, -30.0f),
	/* ⑤a B 前摆；A 不动 */
	R04_STEP(624U, 0.0f, 0.0f, 24.0f, 0.72f,
	          0.0f,  0.0f, 22.0f, 30.0f,  0.0f,  0.0f,
	        -22.0f,-30.0f,  0.0f,  0.0f, 22.0f, -30.0f),
	/* ⑤b B 落地 */
	R04_STEP(496U, 0.30f, 0.0f, 18.0f, 0.64f,
	          0.0f,  0.0f, 22.0f,  0.0f,  0.0f,  0.0f,
	        -22.0f,  0.0f,  0.0f,  0.0f, 22.0f,  0.0f),
	/* ⑥a 全体后蹬前半：从 ⑤b 向「B 中、A 后」插值 */
	R04_STEP(160U, 0.62f, 0.0f, 42.0f, 0.58f,
	        -11.0f,  0.0f, 11.0f,  0.0f,-11.0f,  0.0f,
	        -11.0f,  0.0f, 11.0f,  0.0f, 11.0f,  0.0f),
	/* ⑥b 全体后蹬末：腿2、4、6 水平≈中位；腿1、3、5 水平≈后侧 → 接下一循环 ① */
	R04_STEP(360U, 0.62f, 0.0f, 42.0f, 0.58f,
	        -22.0f,  0.0f,  0.0f,  0.0f,-22.0f,  0.0f,
	         0.0f,  0.0f, 22.0f,  0.0f,  0.0f,  0.0f),
};

/* 与上表步①同参数，但三足 A 水平角为 0（腿1/3/5 奇数轴在中间）：仅首次切入前进时用一步 */
static const R04_ActionStep R04_ForwardCrawlStep1_FromMid =
	R04_STEP(240U, 0.65f, 0.0f, 44.0f, 0.58f,
	          0.0f, 30.0f,  0.0f,  0.0f,  0.0f, 30.0f,
	          0.0f,  0.0f,  0.0f,-30.0f,  0.0f,  0.0f);

static const R04_ActionStep R04_BackwardCrawlSteps[] =
{
	/*  m1     m2     m3     m4     m5     m6     m7     m8     m9    m10    m11    m12  */
	/* 腿号同前进：A=1/3/5，B=2/4/6；后退为另一套前后角符号，结构对称 */

	/* 1: 三足 A 抬起 */
	R04_STEP(200U, 0.65f, 0.0f, 44.0f, 0.58f,
	         18.0f, 36.0f,  0.0f,  0.0f, 18.0f, 36.0f,
	          0.0f,  0.0f,-18.0f,-36.0f,  0.0f,  0.0f),
	/* 2: GroupA swing/land — 落地相软，与 1/3/4/6 高刚度区分 */
	R04_STEP(400U, 0.32f, 0.0f, 18.0f, 0.64f,
	        -18.0f,  0.0f,  0.0f,  0.0f,-18.0f,  0.0f,
	          0.0f,  0.0f, 18.0f,  0.0f,  0.0f,  0.0f),
	/* 3: ALL push forward — 支撑硬 */
	R04_STEP(300U, 0.62f, 0.0f, 42.0f, 0.58f,
	          0.0f,  0.0f, 18.0f,  0.0f,  0.0f,  0.0f,
	        -18.0f,  0.0f,  0.0f,  0.0f,-18.0f,  0.0f),
	/* 4: GroupB lift from front — RL m8 与前进爬行同为 +36 抬腿 */
	R04_STEP(200U, 0.65f, 0.0f, 44.0f, 0.58f,
	          0.0f,  0.0f, 18.0f, 36.0f,  0.0f,  0.0f,
	        -18.0f, 36.0f,  0.0f,  0.0f,-18.0f,-36.0f),
	/* 5: GroupB swing/land — 同 step2 */
	R04_STEP(400U, 0.32f, 0.0f, 18.0f, 0.64f,
	          0.0f,  0.0f,-18.0f,  0.0f,  0.0f,  0.0f,
	         18.0f,  0.0f,  0.0f,  0.0f, 18.0f,  0.0f),
	/* 6: ALL push forward — 支撑硬 */
	R04_STEP(300U, 0.62f, 0.0f, 42.0f, 0.58f,
	         18.0f,  0.0f,  0.0f,  0.0f, 18.0f,  0.0f,
	          0.0f,  0.0f,-18.0f,  0.0f,  0.0f,  0.0f),
};

/* 平移：±18°；整组 kp=36、kd=0.55、speed=0.58（三足承重） */
static const R04_ActionStep R04_StrafeLeftSteps[] =
{
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         -8.0f, 18.0f, -6.0f, 0.0f,  8.0f, 18.0f,
	         -8.0f, 0.0f,  6.0f, 18.0f,  8.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	        -10.0f, 0.0f,  8.0f, 0.0f, 10.0f, 0.0f,
	        -10.0f, 0.0f,  8.0f, 0.0f, 10.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	          8.0f, 0.0f, -6.0f, 18.0f, -8.0f, 0.0f,
	          8.0f, 18.0f, -6.0f, 0.0f, -8.0f, 18.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         10.0f, 0.0f, -8.0f, 0.0f,-10.0f, 0.0f,
	         10.0f, 0.0f, -8.0f, 0.0f,-10.0f, 0.0f),
};

/* 右平移：与左对称，增益同左 */
static const R04_ActionStep R04_StrafeRightSteps[] =
{
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	          8.0f, 18.0f,  6.0f, 0.0f, -8.0f, 18.0f,
	          8.0f, 0.0f, -6.0f, 18.0f, -8.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         10.0f, 0.0f, -8.0f, 0.0f,-10.0f, 0.0f,
	         10.0f, 0.0f, -8.0f, 0.0f,-10.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         -8.0f, 0.0f,  6.0f, 18.0f,  8.0f, 0.0f,
	         -8.0f, 18.0f,  6.0f, 0.0f,  8.0f, 18.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	        -10.0f, 0.0f,  8.0f, 0.0f, 10.0f, 0.0f,
	        -10.0f, 0.0f,  8.0f, 0.0f, 10.0f, 0.0f),
};

/* 原地转圈（Sbus[0]）：六步 tripod + ±36° 抬腿，与爬行同相；左转=前进爬行几何、右转=后退爬行几何 */
static const R04_ActionStep R04_TurnLeftSteps[] =
{
	R04_STEP(200U, 0.65f, 0.0f, 44.0f, 0.58f,
	        -18.0f, 36.0f,  0.0f,  0.0f,-18.0f, 36.0f,
	          0.0f,  0.0f, 18.0f,-36.0f,  0.0f,  0.0f),
	R04_STEP(400U, 0.32f, 0.0f, 18.0f, 0.64f,
	         18.0f,  0.0f,  0.0f,  0.0f, 18.0f,  0.0f,
	          0.0f,  0.0f,-18.0f,  0.0f,  0.0f,  0.0f),
	R04_STEP(300U, 0.62f, 0.0f, 42.0f, 0.58f,
	          0.0f,  0.0f,-18.0f,  0.0f,  0.0f,  0.0f,
	         18.0f,  0.0f,  0.0f,  0.0f, 18.0f,  0.0f),
	R04_STEP(200U, 0.65f, 0.0f, 44.0f, 0.58f,
	          0.0f,  0.0f,-18.0f, 36.0f,  0.0f,  0.0f,
	         18.0f, 36.0f,  0.0f,  0.0f, 18.0f,-36.0f),
	R04_STEP(400U, 0.32f, 0.0f, 18.0f, 0.64f,
	          0.0f,  0.0f, 18.0f,  0.0f,  0.0f,  0.0f,
	        -18.0f,  0.0f,  0.0f,  0.0f,-18.0f,  0.0f),
	R04_STEP(300U, 0.62f, 0.0f, 42.0f, 0.58f,
	        -18.0f,  0.0f,  0.0f,  0.0f,-18.0f,  0.0f,
	          0.0f,  0.0f, 18.0f,  0.0f,  0.0f,  0.0f),
};

static const R04_ActionStep R04_TurnRightSteps[] =
{
	R04_STEP(200U, 0.65f, 0.0f, 44.0f, 0.58f,
	         18.0f, 36.0f,  0.0f,  0.0f, 18.0f, 36.0f,
	          0.0f,  0.0f,-18.0f,-36.0f,  0.0f,  0.0f),
	R04_STEP(400U, 0.32f, 0.0f, 18.0f, 0.64f,
	        -18.0f,  0.0f,  0.0f,  0.0f,-18.0f,  0.0f,
	          0.0f,  0.0f, 18.0f,  0.0f,  0.0f,  0.0f),
	R04_STEP(300U, 0.62f, 0.0f, 42.0f, 0.58f,
	          0.0f,  0.0f, 18.0f,  0.0f,  0.0f,  0.0f,
	        -18.0f,  0.0f,  0.0f,  0.0f,-18.0f,  0.0f),
	R04_STEP(200U, 0.65f, 0.0f, 44.0f, 0.58f,
	          0.0f,  0.0f, 18.0f, 36.0f,  0.0f,  0.0f,
	        -18.0f, 36.0f,  0.0f,  0.0f,-18.0f,-36.0f),
	R04_STEP(400U, 0.32f, 0.0f, 18.0f, 0.64f,
	          0.0f,  0.0f,-18.0f,  0.0f,  0.0f,  0.0f,
	         18.0f,  0.0f,  0.0f,  0.0f, 18.0f,  0.0f),
	R04_STEP(300U, 0.62f, 0.0f, 42.0f, 0.58f,
	         18.0f,  0.0f,  0.0f,  0.0f, 18.0f,  0.0f,
	          0.0f,  0.0f,-18.0f,  0.0f,  0.0f,  0.0f),
};

static const R04_ActionGroup R04_ActionGroups[] =
{
	{R04_StandSteps,         R04_ARRAY_SIZE(R04_StandSteps),         1U},
	{R04_ForwardCrawlSteps,  R04_ARRAY_SIZE(R04_ForwardCrawlSteps),  1U},
	{R04_BackwardCrawlSteps, R04_ARRAY_SIZE(R04_BackwardCrawlSteps), 1U},
	{R04_StrafeLeftSteps,    R04_ARRAY_SIZE(R04_StrafeLeftSteps),    1U},
	{R04_StrafeRightSteps,   R04_ARRAY_SIZE(R04_StrafeRightSteps),   1U},
	{R04_TurnLeftSteps,      R04_ARRAY_SIZE(R04_TurnLeftSteps),      1U},
	{R04_TurnRightSteps,     R04_ARRAY_SIZE(R04_TurnRightSteps),     1U},
};

static R04_ActionRuntime R04_ActionState = {R04_ACTION_STAND, 0U, 0U};
/* 每次从遥控重新切入前进时置 1：第 1 步用“全在中间”抬腿；完成该步后清 0，循环内第 1 步用表中“后方” */
static uint8_t R04_FwdUseMidStep1 = 0U;
static uint16_t R04_RemoteNeutralTick = 0U;
static uint8_t R04_RemoteArmed = 0U;
static uint8_t R04_ControlEnabled = 0U;
static uint8_t R04_ZeroPoseValid = 0U;
static float R04_ZeroPoseDeg[R04_MOTOR_COUNT] = {0.0f};
/* 偶数轴 m2,m4,m6,m8,m10,m12：使能瞬间记录的角度；失能时缓降终点 */
static float R04_EvenPreEnableDeg[6];
static float R04_EvenRampStartDeg[6];
static uint8_t R04_EvenRampActive = 0U;
static uint16_t R04_EvenRampTick = 0U;
static uint8_t R04_LastCtrlForRamp = 0U;
static uint8_t R04_HasHadEnableLift = 0U;
static uint8_t R04_EvenRampEnableActive = 0U;
static uint16_t R04_EvenEnableRampTick = 0U;
static float R04_EvenEnableStartDeg[6];
static float R04_EvenEnableTargetDeg[6];
static uint16_t R04_PostEnableSoftRemainMs = 0U;

static uint8_t R04_UpdateZeroPoseFromFeedback(void)
{
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
	{
		const Rs_Motor *motor = RobStride04_GetMotor(R04_MotorIds[idx]);
		if (motor == 0)
		{
			return 0U;
		}
		R04_ZeroPoseDeg[idx] = R04_RAD2DEG(motor->position);
	}

	R04_ZeroPoseValid = 1U;
	return 1U;
}

static uint8_t R04_IsControlEnabled(void)
{
	if (Sbus_Value[4] > R04_CONTROL_ENABLE_TH)
	{
		R04_ControlEnabled = 1U;
	}
	else if (Sbus_Value[4] < R04_CONTROL_DISABLE_TH)
	{
		R04_ControlEnabled = 0U;
	}

	return R04_ControlEnabled;
}

static void R04_DisableAllMotors(void)
{
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
	{
		RobStride04_Set(R04_MotorIds[idx], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U);
	}
}

static R04_ActionId R04_SelectActionFromRemote(void)
{
	float forward_cmd = Sbus_Value[2];
	/* [0]：原地转圈，>0 左转、<0 右转 */
	float spin_cmd = Sbus_Value[0];
	/* [3]：左右平移（原 [0] 侧移逻辑） */
	float strafe_cmd = Sbus_Value[3];

	if (Sbus_Connect_Flag == 0U)
	{
		R04_RemoteNeutralTick = 0U;
		R04_RemoteArmed = 0U;
		R04_ControlEnabled = 0U;
		return R04_ACTION_STAND;
	}

	if (R04_IsControlEnabled() == 0U)
	{
		R04_RemoteNeutralTick = 0U;
		R04_RemoteArmed = 0U;
		return R04_ACTION_STAND;
	}

	if (R04_RemoteArmed == 0U)
	{
		if (R04_IsRemoteNeutral() == 0U)
		{
			R04_RemoteNeutralTick = 0U;
			return R04_ACTION_STAND;
		}

		if (R04_RemoteNeutralTick < R04_REMOTE_ARM_MS)
		{
			R04_RemoteNeutralTick++;
			return R04_ACTION_STAND;
		}

		R04_RemoteArmed = 1U;
	}

	if (spin_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_TURN_LEFT;
	}

	if (spin_cmd < -R04_CMD_DEADBAND)
	{
		return R04_ACTION_TURN_RIGHT;
	}

	if (strafe_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_STRAFE_RIGHT;
	}

	if (strafe_cmd < -R04_CMD_DEADBAND)
	{
		return R04_ACTION_STRAFE_LEFT;
	}

	if (forward_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_FORWARD_CRAWL;
	}

	if (forward_cmd < -R04_CMD_DEADBAND)
	{
		return R04_ACTION_BACKWARD_CRAWL;
	}

	if ((R04_Abs(forward_cmd) < R04_CMD_DEADBAND) &&
	    (R04_Abs(spin_cmd) < R04_CMD_DEADBAND) &&
	    (R04_Abs(strafe_cmd) < R04_CMD_DEADBAND))
	{
		return R04_ACTION_STAND;
	}

	return R04_ACTION_STAND;
}

static void R04_ResetActionState(R04_ActionId action_id)
{
	R04_ActionState.action_id = action_id;
	R04_ActionState.step_idx = 0U;
	R04_ActionState.step_tick = 0U;
	if (action_id == R04_ACTION_FORWARD_CRAWL)
	{
		R04_FwdUseMidStep1 = 1U;
	}
}

/* 前进步态 step_idx：0~2 为 A 抬/摆/落（B=2、4、6 撑地）；5~7 为 B 抬/摆/落（A=1、3、5 撑地）；其余为六足参与蹬地，不加强单侧 */
static uint8_t R04_ForwardTripodStanceMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if (step_idx <= 2U)
	{
		return ((motor_idx == 2U) || (motor_idx == 3U) || (motor_idx == 6U) || (motor_idx == 7U) ||
		        (motor_idx == 10U) || (motor_idx == 11U)) ? 1U : 0U;
	}
	if ((step_idx >= 5U) && (step_idx <= 7U))
	{
		return ((motor_idx == 0U) || (motor_idx == 1U) || (motor_idx == 4U) || (motor_idx == 5U) ||
		        (motor_idx == 8U) || (motor_idx == 9U)) ? 1U : 0U;
	}
	return 0U;
}

static void R04_ApplyActionStep(const R04_ActionStep *step, R04_ActionId action_id, uint8_t forward_step_idx)
{
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
	{
		float target_deg = step->pos_deg[idx];
#if R04_M11_FORWARD_FB_INVERT
		/* 电机11=pos[10]，站立阶段目标为 0；仅前进步态与实机水平正方向相反时取反 */
		if ((idx == 10U) && (action_id == R04_ACTION_FORWARD_CRAWL))
		{
			target_deg = -target_deg;
		}
#endif
		if (R04_ZeroPoseValid != 0U)
		{
			target_deg += R04_ZeroPoseDeg[idx];
		}

		float kp = step->kp;
		float kd = step->kd;
		if ((action_id == R04_ACTION_FORWARD_CRAWL) && (R04_ForwardTripodStanceMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_FWD_TRIPOD_STANCE_KP_MUL;
			kd *= R04_FWD_TRIPOD_STANCE_KD_MUL;
		}

		RobStride04_Set(R04_MotorIds[idx],
		                R04_DEG2RAD(target_deg),
		                step->speed,
		                step->torque,
		                kp,
		                kd,
		                1);
	}
}

/* Sbus[4] 上升沿：奇数轴零位=当前角；偶数轴先记目标（下压撑起），零位从当前角在 R04_EVEN_ENABLE_RAMP_MS 内慢慢插值过去，避免一使能就把“初始位置”抬高 */
static void R04_OnControlEnableEdge(void)
{
	for (uint8_t i = 0U; i < R04_MOTOR_COUNT; i++)
	{
		const Rs_Motor *motor = RobStride04_GetMotor(R04_MotorIds[i]);
		if (motor == 0)
		{
			return;
		}
		float deg = R04_RAD2DEG(motor->position);
		if ((i & 1U) != 0U)
		{
			uint8_t ek = (uint8_t)(i >> 1);
			R04_EvenPreEnableDeg[ek] = deg;
			R04_EvenEnableStartDeg[ek] = deg;
			if ((i == 1U) || (i == 3U) || (i == 5U))
			{
				R04_EvenEnableTargetDeg[ek] = deg - R04_BODY_LIFT_EVEN_DEG;
			}
			else
			{
				R04_EvenEnableTargetDeg[ek] = deg + R04_BODY_LIFT_EVEN_DEG;
			}
			R04_ZeroPoseDeg[i] = deg;
		}
		else
		{
			R04_ZeroPoseDeg[i] = deg;
		}
	}
	R04_EvenEnableRampTick = 0U;
	R04_EvenRampEnableActive = 1U;
	R04_PostEnableSoftRemainMs = 0U;
	R04_ZeroPoseValid = 1U;
	R04_HasHadEnableLift = 1U;
}

static void R04_RunEvenRampEnable(void)
{
	R04_EvenEnableRampTick++;
	float alpha = (float)R04_EvenEnableRampTick / (float)R04_EVEN_ENABLE_RAMP_MS;
	if (alpha > 1.0f)
	{
		alpha = 1.0f;
	}
	/* smootherstep：比 smoothstep 起停更平，末端更柔 */
	float t = alpha;
	float u = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
	for (uint8_t i = 1U; i < R04_MOTOR_COUNT; i += 2U)
	{
		uint8_t ek = (uint8_t)(i >> 1);
		float s = R04_EvenEnableStartDeg[ek];
		float t = R04_EvenEnableTargetDeg[ek];
		R04_ZeroPoseDeg[i] = s + (t - s) * u;
	}
	if (alpha >= 1.0f)
	{
		R04_EvenRampEnableActive = 0U;
		R04_PostEnableSoftRemainMs = R04_POST_ENABLE_SOFT_MS;
	}
}

static void R04_OnDisableStartRamp(void)
{
	R04_EvenRampActive = 1U;
	R04_EvenRampTick = 0U;
	for (uint8_t i = 1U; i < R04_MOTOR_COUNT; i += 2U)
	{
		const Rs_Motor *motor = RobStride04_GetMotor(R04_MotorIds[i]);
		if (motor == 0)
		{
			R04_EvenRampActive = 0U;
			return;
		}
		uint8_t ek = (uint8_t)(i >> 1);
		R04_EvenRampStartDeg[ek] = R04_RAD2DEG(motor->position);
	}
}

/* 仅偶数轴插值到使能前角度；奇数轴每帧保持当前反馈，步态表与其它逻辑不变 */
static void R04_RunEvenRampDown(void)
{
	R04_EvenRampTick++;
	float alpha = (float)R04_EvenRampTick / (float)R04_EVEN_RAMP_DOWN_MS;
	if (alpha > 1.0f)
	{
		alpha = 1.0f;
	}
	/* smoothstep 缓变目标，减轻失能落地末段冲击 */
	float u = alpha * alpha * (3.0f - 2.0f * alpha);

	/* 略降 kp、略增 kd，末段更柔；轨迹仍慢 */
	const float rs = 0.52f;
	const float rt = 0.0f;
	const float rkp = 22.0f;
	const float rkd = 0.55f;

	for (uint8_t i = 0U; i < R04_MOTOR_COUNT; i++)
	{
		const Rs_Motor *motor = RobStride04_GetMotor(R04_MotorIds[i]);
		if (motor == 0)
		{
			return;
		}
		float target_deg;
		if ((i & 1U) != 0U)
		{
			uint8_t ek = (uint8_t)(i >> 1);
			float e0 = R04_EvenRampStartDeg[ek];
			float e1 = R04_EvenPreEnableDeg[ek];
			target_deg = e0 + (e1 - e0) * u;
		}
		else
		{
			target_deg = R04_RAD2DEG(motor->position);
		}
		RobStride04_Set(R04_MotorIds[i], R04_DEG2RAD(target_deg), rs, rt, rkp, rkd, 1U);
	}

	if (alpha >= 1.0f)
	{
		R04_EvenRampActive = 0U;
		R04_HasHadEnableLift = 0U;
		R04_ResetActionState(R04_ACTION_STAND);
		R04_RemoteNeutralTick = 0U;
		R04_RemoteArmed = 0U;
		(void)R04_UpdateZeroPoseFromFeedback();
		R04_DisableAllMotors();
	}
}
		
void System_Init()
{
	
//	HAL_Delay(5000);//???5S????????
//	
	
  Remote_Init(); //??????????
	Step_Motor_Init();//????????????
	Relay_Init();//??????????
	Pwm_Init(); //PWM?????
//	NSM_77_Init();//????????????
	RS485_Init();//RS485?????
	BRT38_Init();//??????????????
	
	YS_Motor_Init();//????????????
	
  WS2812B_Init(&htim3, &hdma_tim3_ch1);  //bling 
	
	
	Step_Pwm_Fre(5000);  //??????????????????
	Servo_Init();//????????
	
	Sbus_Init();
	//Ibus_Init();
	
	R04_filter(&hfdcan1,1);
	RobStride04_Manager_Init(R04_MotorIds, (uint8_t)(sizeof(R04_MotorIds) / sizeof(R04_MotorIds[0])));

  RM3508_Init();  //3508????????
	

	HAL_GPIO_WritePin(IN_1_GPIO_Port,IN_1_Pin,GPIO_PIN_RESET);  //A???
	HAL_GPIO_WritePin(IN_2_GPIO_Port,IN_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_3_GPIO_Port,IN_3_Pin,GPIO_PIN_RESET);  //B???
	HAL_GPIO_WritePin(IN_4_GPIO_Port,IN_4_Pin,GPIO_PIN_RESET);
	
	

	PID_struct_init(&Motor1_Pid,POSITION_PID,5.0f,0.2f,5.5f,0.005f,0.5f,10.0f,0.02f); //PID?????
	
	PID_struct_init(&YS_Motor1_Pid,POSITION_PID,5.0f,0.2f,5.5f,0.005f,0.5f,10.0f,0.02f); //PID?????	

	HAL_TIM_Base_Start_IT(&htim6); //所有外设初始化完成后，再启动1ms任务定时器
	
	
	
}





/********************************************************************************************************
Function Name: Mymain  ??????main????
Author       : ZFY
Date         : 2023-06-09
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
									
			
											
void Mymain()
{
	
	System_Init();

			for(;;)
			{

				HAL_GPIO_TogglePin(Led_1_GPIO_Port,Led_1_Pin);
	
                WS2812B_RainbowEffect(&htim3, &hdma_tim3_ch1,1.0f);

			

      }

	
	
}




/********************************************************************************************************
Function Name: Millisecond_Task  ????????
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/





float Speed_Set=0.0f;

uint8_t Key_Flag=0;	
uint8_t Key_Down_Flag=0;	

float torque_set=0.0f;

								
extern Rs_Motor Rs_Motor_1;   //?????????1



uint8_t Lock_Flag=0x01;
int i=0;

uint8_t Motor_id=0x01;
float P_des=0.0f;  //???? ??????????+5.7????
float V_des=0.0f;
float T_ff=0.0f;
float Kp=0.5f;   //????  ??????3.5  ???????????0.1
float Kd=0.02f;
float Zero_Offset=0.0f;

float Pos_Set=0.0f;

float Pos_InV=-0.0f;

//void MS_Task()
//{
//	
//	if(Sbus_Value[7]<=0.0f) 
//	{
//		i++;
//		if(i>1000)
//		{
//		  YS_Set(Motor_id,0.0f,0.0f,0.0f,0.0f,0.0f,0);
//			Zero_Offset=Motor_2.pfb;//??????????????
//			i=0;    //1S????????? ????????????
//		}
//	
//	}else
//	{
//
//		
//	 YS_Set(Motor_id,P_des,V_des,T_ff,Kp,Kd,1);
//	
//	}
//	
//}

void MS_Task()
{
	R04_ActionId selected_action;
	const R04_ActionGroup *group;
	const R04_ActionStep *step;

	if (RobStride04_IsReady() == 0U)
	{
		return;
	}

	if (Sbus_Connect_Flag == 0U)
	{
		R04_EvenRampActive = 0U;
		R04_EvenRampEnableActive = 0U;
		R04_PostEnableSoftRemainMs = 0U;
		R04_LastCtrlForRamp = 0U;
		R04_HasHadEnableLift = 0U;
		R04_ResetActionState(R04_ACTION_STAND);
		R04_RemoteNeutralTick = 0U;
		R04_RemoteArmed = 0U;
		(void)R04_UpdateZeroPoseFromFeedback();
		R04_DisableAllMotors();
		return;
	}

	{
		uint8_t ctrl_now = R04_IsControlEnabled();

		if ((ctrl_now == 0U) && (R04_EvenRampActive != 0U))
		{
			R04_RunEvenRampDown();
			return;
		}

		if (ctrl_now == 0U)
		{
			R04_EvenRampEnableActive = 0U;
			R04_PostEnableSoftRemainMs = 0U;
			if (R04_LastCtrlForRamp != 0U)
			{
				if (R04_HasHadEnableLift != 0U)
				{
					R04_OnDisableStartRamp();
					R04_LastCtrlForRamp = 0U;
					/* 首帧立即下发缓降指令；若只 return，重力会在 1ms 空档里把机身拉塌 */
					if (R04_EvenRampActive != 0U)
					{
						R04_RunEvenRampDown();
					}
					return;
				}
			}
			R04_LastCtrlForRamp = 0U;
			R04_ResetActionState(R04_ACTION_STAND);
			R04_RemoteNeutralTick = 0U;
			R04_RemoteArmed = 0U;
			(void)R04_UpdateZeroPoseFromFeedback();
			R04_DisableAllMotors();
			return;
		}

		if (R04_LastCtrlForRamp == 0U)
		{
			R04_OnControlEnableEdge();
		}
		R04_LastCtrlForRamp = 1U;

		if (R04_EvenRampEnableActive != 0U)
		{
			R04_RunEvenRampEnable();
			R04_ApplyActionStep(&R04_StandStepsEnableSoft, R04_ACTION_STAND, 0U);
			return;
		}

		if (R04_PostEnableSoftRemainMs > 0U)
		{
			R04_PostEnableSoftRemainMs--;
			R04_ApplyActionStep(&R04_StandStepsEnableSoft, R04_ACTION_STAND, 0U);
			return;
		}
	}

	if ((R04_ZeroPoseValid == 0U) && (R04_UpdateZeroPoseFromFeedback() == 0U))
	{
		R04_DisableAllMotors();
		return;
	}

	selected_action = R04_SelectActionFromRemote();
	if (selected_action != R04_ActionState.action_id)
	{
		R04_ResetActionState(selected_action);
	}

	if (selected_action == R04_ACTION_STAND)
	{
		/* 摇杆回中：目标为相对零位全 0，回到上电/校准时的中位，而不是停在爬行中途姿态 */
		R04_ApplyActionStep(&R04_StandSteps[0], R04_ACTION_STAND, 0U);
		return;
	}

	group = &R04_ActionGroups[selected_action];
	if (group->step_count == 0U)
	{
		return;
	}

	step = &group->steps[R04_ActionState.step_idx];
	if ((selected_action == R04_ACTION_FORWARD_CRAWL) && (R04_ActionState.step_idx == 0U) && (R04_FwdUseMidStep1 != 0U))
	{
		step = &R04_ForwardCrawlStep1_FromMid;
	}
	{
		uint8_t fwd_idx = 0U;
		if (selected_action == R04_ACTION_FORWARD_CRAWL)
		{
			fwd_idx = R04_ActionState.step_idx;
		}
		R04_ApplyActionStep(step, selected_action, fwd_idx);
	}

	R04_ActionState.step_tick++;
	if (R04_ActionState.step_tick >= step->hold_ms)
	{
		R04_ActionState.step_tick = 0U;
		if ((R04_ActionState.step_idx + 1U) < group->step_count)
		{
			if ((selected_action == R04_ACTION_FORWARD_CRAWL) && (R04_ActionState.step_idx == 0U) && (R04_FwdUseMidStep1 != 0U))
			{
				R04_FwdUseMidStep1 = 0U;
			}
			R04_ActionState.step_idx++;
		}
		else if (group->loop != 0U)
		{
			R04_ActionState.step_idx = 0U;
		}
	}
}











void Millisecond_Task()      //1ms???????
{


	
	MS_Task();
	RobStride04_Manager_Task1ms();
	
	



}

/********************************************************************************************************
Function Name: Millisecond_10_Task  10????????
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/
void Millisecond_50_Task()      //10ms???????
{
	
//	Position_Read(0x01,0x0000,0x0002);//??????????

}

/********************************************************************************************************
Function Name: HAL_TIM_PeriodElapsedCallback  ????????????
Author       : ZFY
Date         : 2024-01-05
Description  :
Outputs      : void
Notes        :
********************************************************************************************************/

int pluse=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == TIM6)
	{
		Millisecond_Task();  //1ms????
		pluse++;
		if(pluse>=50)
		{
			Millisecond_50_Task();//50ms????
			pluse=0;
		}
	}
}
















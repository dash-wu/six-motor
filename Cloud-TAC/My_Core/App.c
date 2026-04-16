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
/*
 * 左右动作一致性（设计约定，改增益/表时请一起核对）：
 * - 遥控：同一 R04_CMD_DEADBAND；前进/后退、左转/右转、左平移/右平移 成对对称（通道相反、动作镜像）。
 * - 前进/后退：同一套 MIT 步序与叠乘宏；后退表内水平已取反，与前进力度对称。
 * - 原地转：左转/右转共用 R04_TurnInPlaceLeftSteps、同一 R04_TURN_PLANAR_DEG；右转仅在 R04_StepTargetDeg
 *   对全部水平 fb（偶 motor_idx）取反，竖直 ud 不取反，且转向中不叠 R04_M11_FORWARD_FB_INVERT（免腿6左右转不对称）。
 * - 侧平移：R04_StrafeRightSteps 为 R04_StrafeLeftSteps 的左右镜像，增益相同。
 * - ②a/⑤a 单腿竖直加强：②a 仅腿2(ud 序号 3)、⑤a 仅腿5(ud 序号 9)，左中与右中成对，共用同一组 SWING_SINGLE_SIDE 倍率。
 * - ①/④ 抬腿：B 三足 ud(3,7,11) 与 A 三足 ud(1,5,9) 对称加强。
 */
/* 前进/后退：三足撑地叠乘；略低于曾用过的高刚度，整体更接近转向的「轻」手感 */
#define R04_FWD_TRIPOD_STANCE_KP_MUL 1.58f
#define R04_FWD_TRIPOD_STANCE_KD_MUL 1.26f
/* 撑地腿竖直(ud)轴再略顶重力（奇数下标 m2,m4…）；与上行叠乘 */
#define R04_FWD_TRIPOD_STANCE_UD_KP_MUL 1.04f
#define R04_FWD_TRIPOD_STANCE_UD_KD_MUL 1.04f
/* 前摆相 ②a / ⑤a：对侧三足水平前摆、重心前移，仅再加强撑地腿「上下轴」刚度，减轻该段主体下坠 */
#define R04_FWD_SWING_STANCE_UD_KP_EXTRA 1.10f
#define R04_FWD_SWING_STANCE_UD_KD_EXTRA 1.04f
/* ②a/⑤a 前摆平移：三条撑地腿 ud 整体再略加（与 SWING_STANCE_UD_EXTRA 叠乘；不含抬腿①④） */
#define R04_FWD_SWING_TRANS_TRIPOD_UD_KP_MUL 1.04f
#define R04_FWD_SWING_TRANS_TRIPOD_UD_KD_MUL 1.04f
/* 同上平移相：左右仅单足一侧竖直轴再叠乘（与 R04_FWD_SWING_SINGLE_SIDE_UD_IDX_* 对应） */
#define R04_FWD_SWING_SINGLE_SIDE_UD_KP_MUL 1.14f
#define R04_FWD_SWING_SINGLE_SIDE_UD_KD_MUL 1.10f
#define R04_FWD_SWING_SINGLE_SIDE_UD_IDX_STEP1 3U /* ②a：腿2(左中) 单撑加强，与 STEP6 成对 */
#define R04_FWD_SWING_SINGLE_SIDE_UD_IDX_STEP6 9U /* ⑤a：腿5(右中) 单撑加强；倍率与 STEP1 必相同 */
/* ②b(step2)/⑤b(step7) 摆动足落地相：落足腿再叠乘；kp 更软、kd 吃速度，减轻砸地声 */
#define R04_FWD_LANDING_KP_MUL          0.48f
#define R04_FWD_LANDING_KD_MUL          2.08f
#define R04_FWD_LANDING_UD_KD_MUL       1.82f
#define R04_FWD_LANDING_TOUCH_UD_KD_MUL 1.42f
/* ②b/⑤b：摆腿落地时对侧撑地 ud 略顶即可；过大则整机发「死」、声大 */
#define R04_FWD_LANDING_STANCE_UD_KP_MUL  1.28f
#define R04_FWD_LANDING_STANCE_UD_KD_MUL  1.16f
/* 同上：步前半段负载突变略顶 */
#define R04_FWD_LANDING_STANCE_INSTANT_UD_KP_MUL  1.08f
#define R04_FWD_LANDING_STANCE_INSTANT_UD_KD_MUL  1.06f
/* 原地转向：落地摆腿略软于前进；过小则空中平移后落地机身顶不起来 */
#define R04_TURN_LANDING_KP_EXTRA_MUL       0.93f
#define R04_TURN_LANDING_KD_EXTRA_MUL       1.10f
#define R04_TURN_LANDING_TOUCH_UD_KD_EXTRA_MUL 1.12f
/* ①(step0)/④(step5) 抬腿相：对侧三足撑地 ud 略加强以抑下沉；过大易「地上腿往上顶」，实机可再调 */
#define R04_FWD_TRIPOD_LIFT_STANCE_UD_KP_MUL   1.08f
#define R04_FWD_TRIPOD_LIFT_STANCE_UD_KD_MUL   1.08f
#define R04_FWD_TRIPOD_LIFT_INSTANT_UD_KP_MUL  1.04f
#define R04_FWD_TRIPOD_LIFT_INSTANT_UD_KD_MUL  1.04f
/* ③④⑥（step3~4、8~9）六足同时撑地：全部 ud 略降 kp、略升 kd，减轻竖直「上顶」、起伏反差 */
#define R04_FWD_HEX_SUPPORT_UD_KP_MUL  0.86f
#define R04_FWD_HEX_SUPPORT_UD_KD_MUL  1.15f
/* 前摆 ②a/⑤a：摆动腿水平/竖直轴降 kp、升 kd；步后半段再抬 kd，抑制到位惯性晃 */
#define R04_SWING_FB_OSC_KP_MUL       (0.70f)
#define R04_SWING_FB_OSC_KD_MUL       (1.58f)
#define R04_SWING_FB_OSC_END_KD_MUL   (1.32f)
#define R04_SWING_UD_OSC_KP_MUL       (0.86f)
#define R04_SWING_UD_OSC_KD_MUL       (1.28f)
#define R04_SWING_UD_OSC_END_KD_MUL   (1.18f)
/* 抬腿相 ①/④：抬起侧三足竖直轴到位易上下抖，略降 kp、升 kd，步后半段再抬 kd */
#define R04_LIFT_UD_OSC_KP_MUL        (0.82f)
#define R04_LIFT_UD_OSC_KD_MUL        (1.36f)
#define R04_LIFT_UD_OSC_END_KD_MUL    (1.24f)
/* 前进：最短保持 hold_ms 后，若反馈与目标仍差较大则延后切步，减轻「上步未完成就抢下一步」；超时后强制切步防卡死 */
#define R04_FWD_SETTLE_WAIT         1
#define R04_FWD_SETTLE_ERR_DEG      (5.0f)
#define R04_FWD_SETTLE_EXTRA_MS     (42U)
/* 爬行抬腿竖直目标角幅值（度）；略小抬足更低，过小易拖地 */
#define R04_CRAWL_LIFT_UD_DEG       (24.0f)
/* 爬行水平摆幅（度）；③⑥ 蹬地插值为半幅（≈ SWING/2） */
#define R04_CRAWL_SWING_FB_DEG       (34.0f)
#define R04_CRAWL_PUSH_HALF_FB_DEG (17.0f)
/* ①/④ 抬腿时撑地足水平偏置（随 SWING 28→34 同比） */
#define R04_CRAWL_LIFT_STANCE_FB_DEG (28.0f)
/* 原地转向：平移式；每步保持时间（过短易跟不上，可按实机微调） */
#define R04_TURN_SIMPLE_STEP_MS      (175U)
#define R04_TURN_SETTLE_EXTRA_MS     (12U) /* 转向专用到位等待上限增量，与 FWD_SETTLE 叠用 */
/*
 * 原地转向水平角 P（度），与前进爬行 R04_CRAWL_SWING_FB_DEG 无关。
 * 约定 P=30：三足 A 抬起 → 空中水平至 +P → 落地；六足同时水平 −P「蹬」→ A 回 0°、B 至 −P；
 * 再抬 B（在 −P）→ 空中至 +P → 落地；再六足 −P → B 回 0°、A 至 −P，循环。
 * 仅改此处即可改平移幅度。
 */
#define R04_TURN_PLANAR_DEG          (30.0f)
/* 摇杆回中后等待多久才允许步态；0=使能后立即响应（仅 Sbus[4] 与回连有效时仍需回中） */
#define R04_REMOTE_ARM_MS 0U
#define R04_CONTROL_DISABLE_TH (-0.5f)
#define R04_CONTROL_ENABLE_TH  (0.5f)
/* Sbus[4] 使能后偶数轴相对使能前再“下压”撑起机身（度），符号可按实机反向改 */
/* 使能瞬间偶数轴相对“趴地”姿态多压下的角度；越大底盘离地越高，过小撑不起来，可实机微调（仅改高度，不改 MIT 力度） */
#define R04_BODY_LIFT_EVEN_DEG (30.0f)
/* 失能后：各轴目标从当前角随时间线性变化，偶数轴(ud)落到使能前趴地角 R04_EvenPreEnableDeg；时长 ms（与 1ms 任务一致） */
#define R04_EVEN_RAMP_DOWN_MS  (5000U)
/* 跟踪上述角度轨迹时的 MIT 参数（全程不变，靠目标角线性落下） */
#define R04_DISABLE_RAMP_KP    (28.0f)
#define R04_DISABLE_RAMP_KD      (0.56f)
#define R04_DISABLE_RAMP_SPEED   (0.54f)
/* 使能后偶数轴零位插值到撑起目标；过短易“砸地”，过长响应慢 */
#define R04_EVEN_ENABLE_RAMP_MS (4000U)
/* 插值结束后仍略软站立若干 ms 再切常规站立；已加强软站立增益，可略缩短 */
#define R04_POST_ENABLE_SOFT_MS (600U)
/* 腿6=电机11/12：仅前进爬行可单独处理符号。若仅腿6「水平」反了：开关 R04_M11_FORWARD_FB_INVERT；若「竖直」反了：开 R04_M12_FORWARD_UD_INVERT */
#define R04_M11_FORWARD_FB_INVERT 1
#define R04_M12_FORWARD_UD_INVERT 0
/* 仅腿6(m11/m12)微调；若改左右对称，优先查机械同构再动 R04_LEG6_*（仅一侧会引入左右差） */
#define R04_LEG6_FB_TRIM_DEG (0.0f)
#define R04_LEG6_UD_TRIM_DEG (0.0f)

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

/* 原地转向：专用 10 步偏航几何 + 与爬行同一套 MIT 叠乘/hold/kp */
#define R04_ACTION_USES_CRAWL_MIT(_id) \
	(((_id) == R04_ACTION_FORWARD_CRAWL) || ((_id) == R04_ACTION_BACKWARD_CRAWL) || \
	 ((_id) == R04_ACTION_TURN_LEFT) || ((_id) == R04_ACTION_TURN_RIGHT))

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

/*
 * 观察步态用：每步保持时间 × 该倍率，轨迹 speed ÷ 该倍率（与表内数值无关）。
 * 改回 1U 即恢复「当前表定」速度与步频。
 */
#define R04_GAIT_OBSERVE_SLOWDOWN_MUL  10U
/* Sbus[5]：[-1,1] 映射为步态速度倍率 [1,8]（最小 1×、最大 8×）；与 OBSERVE 叠乘 */
#define R04_GAIT_SBUS_SPEED_CH 5

static float R04_GaitSpeedMulFromSbus(void)
{
	float v = Sbus_Value[R04_GAIT_SBUS_SPEED_CH];
	if (v < -1.0f)
	{
		v = -1.0f;
	}
	else if (v > 1.0f)
	{
		v = 1.0f;
	}
	return 1.0f + (v + 1.0f) * 3.5f;
}

/* 左转/右转：在 Sbus[5] 倍率上再 ×1.2，比前进/平移约快 20% */
#define R04_TURN_GAIT_SPEED_BOOST_MUL  1.2f

static float R04_GaitUserMulForAction(R04_ActionId action_id, float sbus_mul)
{
	if ((action_id == R04_ACTION_TURN_LEFT) || (action_id == R04_ACTION_TURN_RIGHT))
	{
		return sbus_mul * R04_TURN_GAIT_SPEED_BOOST_MUL;
	}
	return sbus_mul;
}

static uint32_t R04_StepHoldMsScaled(const R04_ActionStep *step, float gait_user_mul)
{
	float t = (float)step->hold_ms * (float)R04_GAIT_OBSERVE_SLOWDOWN_MUL / gait_user_mul;
	if (t < 1.0f)
	{
		t = 1.0f;
	}
	return (uint32_t)(t + 0.5f);
}

static float R04_StepSpeedScaled(float speed, float gait_user_mul)
{
	return speed * gait_user_mul / (float)R04_GAIT_OBSERVE_SLOWDOWN_MUL;
}

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
	 * 水平角约定：A/B 前摆同幅 ±30°（腿5/腿4 等为左右镜像）；蹬伸末 A=0/B=−30 与 B=0/A=−30 交替；③a/⑥a 为起止中点
	 * 共 10 步 loop；首帧 FromMid ① 仍水平中位
	 * hold：再减半，步频再约×2；③ 仅 19ms 极易跟不上，实机可单独加长
	 */

	/* ① A 抬：水平保持 −30°（与⑥b 末一致）；B 不动 */
	R04_STEP(26U, 1.0f, 0.0f, 36.0f, 0.56f,
	        -30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-30.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, 30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②a A 空中至 +30°（腿5 镜像 −30°） */
	R04_STEP(70U, 0.0f, 0.0f, 18.0f, 0.78f,
	         30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, 30.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,-30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②b A 落地 30° */
	R04_STEP(66U, 0.20f, 0.0f, 9.5f, 1.18f,
	         30.0f,  0.0f,  0.0f,  0.0f, 30.0f,  0.0f,
	          0.0f,  0.0f,-30.0f,  0.0f,  0.0f,  0.0f),
	/* ③a 六腿一起后蹬插值中点（相对 ②b→③b） */
	R04_STEP(19U, 1.0f, 0.0f, 36.0f, 0.56f,
	         15.0f,  0.0f,-15.0f,  0.0f, 15.0f,  0.0f,
	         15.0f,  0.0f,-15.0f,  0.0f,-15.0f,  0.0f),
	/* ③b A→0°，B→−30°（镜像） */
	R04_STEP(19U, 1.0f, 0.0f, 36.0f, 0.56f,
	          0.0f,  0.0f,-30.0f,  0.0f,  0.0f,  0.0f,
	         30.0f,  0.0f,  0.0f,  0.0f,-30.0f,  0.0f),
	/* ④ B 抬；姿态同③b */
	R04_STEP(26U, 1.0f, 0.0f, 36.0f, 0.56f,
	          0.0f,  0.0f,-30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	         30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-30.0f, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤a B 前摆至 30°（与②a A 同幅；镜像） */
	R04_STEP(70U, 0.0f, 0.0f, 18.0f, 0.78f,
	          0.0f,  0.0f, 30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	        -30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, 30.0f, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤b B 落地 30° */
	R04_STEP(66U, 0.20f, 0.0f, 9.5f, 1.18f,
	          0.0f,  0.0f, 30.0f,  0.0f,  0.0f,  0.0f,
	        -30.0f,  0.0f,  0.0f,  0.0f, 30.0f,  0.0f),
	/* ⑥a 六腿一起后蹬插值中点（⑤b B=±30 → ⑥b B=0，与 A 中点一致） */
	R04_STEP(19U, 1.0f, 0.0f, 36.0f, 0.56f,
	        -15.0f,  0.0f, 15.0f,  0.0f,-15.0f,  0.0f,
	        -15.0f,  0.0f, 15.0f,  0.0f, 15.0f,  0.0f),
	/* ⑥b B→0°，A→−30°（镜像）→ 接① */
	R04_STEP(38U, 1.0f, 0.0f, 36.0f, 0.56f,
	        -30.0f,  0.0f,  0.0f,  0.0f,-30.0f,  0.0f,
	          0.0f,  0.0f, 30.0f,  0.0f,  0.0f,  0.0f),
};

/* 与上表步①同参数，但三足 A 水平角为 0；首次切入前进/后退共用这一步 */
static const R04_ActionStep R04_ForwardCrawlStep1_FromMid =
	R04_STEP(26U, 1.0f, 0.0f, 36.0f, 0.56f,
	          0.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,  0.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f);

/* 转向切入：三足 A 水平全 0 起抬（仅首次/重入遥控用 R04_CrawlUseMidStep1） */
static const R04_ActionStep R04_TurnYawStep1_FromMid =
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.38f, 0.0f, 38.0f, 0.57f,
	          0.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,  0.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f);

/* 后退爬行：与 R04_ForwardCrawlSteps 同结构、同 hold/kp；水平轴(fb,奇数下标)取反，竖直(ud)不变 → 机体沿反方向爬行 */
static const R04_ActionStep R04_BackwardCrawlSteps[] =
{
	/*  m1     m2     m3     m4     m5     m6     m7     m8     m9    m10    m11    m12  */
	/* ①（前进水平取反） */
	R04_STEP(26U, 1.0f, 0.0f, 36.0f, 0.56f,
	         30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, 30.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,-30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②a */
	R04_STEP(70U, 0.0f, 0.0f, 18.0f, 0.78f,
	        -30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-30.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, 30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②b */
	R04_STEP(66U, 0.20f, 0.0f, 9.5f, 1.18f,
	        -30.0f,  0.0f,  0.0f,  0.0f,-30.0f,  0.0f,
	          0.0f,  0.0f, 30.0f,  0.0f,  0.0f,  0.0f),
	/* ③a */
	R04_STEP(19U, 1.0f, 0.0f, 36.0f, 0.56f,
	        -15.0f,  0.0f, 15.0f,  0.0f,-15.0f,  0.0f,
	        -15.0f,  0.0f, 15.0f,  0.0f, 15.0f,  0.0f),
	/* ③b */
	R04_STEP(19U, 1.0f, 0.0f, 36.0f, 0.56f,
	          0.0f,  0.0f, 30.0f,  0.0f,  0.0f,  0.0f,
	        -30.0f,  0.0f,  0.0f,  0.0f, 30.0f,  0.0f),
	/* ④ */
	R04_STEP(26U, 1.0f, 0.0f, 36.0f, 0.56f,
	          0.0f,  0.0f, 30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	        -30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, 30.0f, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤a */
	R04_STEP(70U, 0.0f, 0.0f, 18.0f, 0.78f,
	          0.0f,  0.0f,-30.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	         30.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-30.0f, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤b */
	R04_STEP(66U, 0.20f, 0.0f, 9.5f, 1.18f,
	          0.0f,  0.0f,-30.0f,  0.0f,  0.0f,  0.0f,
	         30.0f,  0.0f,  0.0f,  0.0f,-30.0f,  0.0f),
	/* ⑥a（与前进 ⑥a 水平 fb 取反） */
	R04_STEP(19U, 1.0f, 0.0f, 36.0f, 0.56f,
	         15.0f,  0.0f,-15.0f,  0.0f, 15.0f,  0.0f,
	         15.0f,  0.0f,-15.0f,  0.0f,-15.0f,  0.0f),
	/* ⑥b */
	R04_STEP(38U, 1.0f, 0.0f, 36.0f, 0.56f,
	         30.0f,  0.0f,  0.0f,  0.0f, 30.0f,  0.0f,
	          0.0f,  0.0f,-30.0f,  0.0f,  0.0f,  0.0f),
};

/*
 * 原地左转（水平几何，P=R04_TURN_PLANAR_DEG，默认 30°）：
 *  0 起抬 A（循环步 0：A 已在 −P）；首帧 FromMid 时 A 自 0° 起抬；
 *  1 空中 A 水平至 +P；2 A 落地 +P；
 *  3 六足水平各 −P（蹬）→ A→0°、B→−P；
 *  4 抬 B；5 空中 B 自 −P 至 +P；6 B 落地 +P；
 *  7 六足各 −P → B→0°、A→−P，接步 0。
 * 右转：同表，R04_StepTargetDeg 对 fb 取反。MIT 步 0..7 → fwd 0,1,2,4,5,6,7,9。
 */
static const R04_ActionStep R04_TurnInPlaceLeftSteps[] =
{
	/* 0 MIT0：循环用（步 7 后 A 在 -P 起抬）；首次步 0 用 R04_TurnYawStep1_FromMid */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.52f, 0.0f, 38.0f, 0.57f,
	         -R04_TURN_PLANAR_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, -R04_TURN_PLANAR_DEG, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, -R04_TURN_PLANAR_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* 1 MIT1：A 空中水平 +P（speed/kp 同前进 ②a：0 / 20） */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.0f, 0.0f, 22.0f, 0.82f,
	         R04_TURN_PLANAR_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, R04_TURN_PLANAR_DEG, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, R04_TURN_PLANAR_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* 2 MIT2：A 落地 +P（基底同前进 ②b，便于撑地腿叠乘后顶起机身） */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.48f, 0.0f, 11.0f, 1.05f,
	         R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f, R04_TURN_PLANAR_DEG,  0.0f,
	          0.0f,  0.0f, R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f),
	/* 3 MIT4：各腿水平 fb 均 −P（speed/kp/kd 同前进 ③b；fb 抑振见 R04_TurnPlanarShiftFbOscMotor） */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 1.0f, 0.0f, 42.0f, 0.58f,
	          0.0f,  0.0f, -R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f,
	         -R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f, -R04_TURN_PLANAR_DEG,  0.0f),
	/* 4 MIT5：抬 B；竖直同前进④ */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.52f, 0.0f, 38.0f, 0.57f,
	          0.0f,  0.0f, -R04_TURN_PLANAR_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	         -R04_TURN_PLANAR_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, -R04_TURN_PLANAR_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* 5 MIT6：B 空中水平 +P（同前进 ⑤a：0 / 20） */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.0f, 0.0f, 22.0f, 0.82f,
	          0.0f,  0.0f, R04_TURN_PLANAR_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	         R04_TURN_PLANAR_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, R04_TURN_PLANAR_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* 6 MIT7：B 落地 +P（同 ②b 基底） */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.48f, 0.0f, 11.0f, 1.05f,
	          0.0f,  0.0f, R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f,
	         R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f, R04_TURN_PLANAR_DEG,  0.0f),
	/* 7 MIT9：六足再各 −P（同前进 ⑥b + 全腿 fb 抑振） */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 1.0f, 0.0f, 42.0f, 0.58f,
	         -R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f, -R04_TURN_PLANAR_DEG,  0.0f,
	          0.0f,  0.0f, -R04_TURN_PLANAR_DEG,  0.0f,  0.0f,  0.0f),
};

/* 平移：水平约 ±14°/±18°、竖直抬足约 26°；整组 kp=36、kd=0.55、speed=0.58（三足承重） */
static const R04_ActionStep R04_StrafeLeftSteps[] =
{
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	        -14.0f, 26.0f,-10.0f, 0.0f, 14.0f, 26.0f,
	        -14.0f, 0.0f, 10.0f, 26.0f, 14.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	        -18.0f, 0.0f, 14.0f, 0.0f, 18.0f, 0.0f,
	        -18.0f, 0.0f, 14.0f, 0.0f, 18.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         14.0f, 0.0f,-10.0f, 26.0f,-14.0f, 0.0f,
	         14.0f, 26.0f,-10.0f, 0.0f,-14.0f, 26.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         18.0f, 0.0f,-14.0f, 0.0f,-18.0f, 0.0f,
	         18.0f, 0.0f,-14.0f, 0.0f,-18.0f, 0.0f),
};

/* 右平移：与左对称，增益同左 */
static const R04_ActionStep R04_StrafeRightSteps[] =
{
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         14.0f, 26.0f, 10.0f, 0.0f,-14.0f, 26.0f,
	         14.0f, 0.0f,-10.0f, 26.0f,-14.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	         18.0f, 0.0f,-14.0f, 0.0f,-18.0f, 0.0f,
	         18.0f, 0.0f,-14.0f, 0.0f,-18.0f, 0.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	        -14.0f, 0.0f, 10.0f, 26.0f, 14.0f, 0.0f,
	        -14.0f, 26.0f, 10.0f, 0.0f, 14.0f, 26.0f),
	R04_STEP(280U, 0.58f, 0.0f, 36.0f, 0.55f,
	        -18.0f, 0.0f, 14.0f, 0.0f, 18.0f, 0.0f,
	        -18.0f, 0.0f, 14.0f, 0.0f, 18.0f, 0.0f),
};

static const R04_ActionGroup R04_ActionGroups[] =
{
	{R04_StandSteps,         R04_ARRAY_SIZE(R04_StandSteps),         1U},
	{R04_ForwardCrawlSteps,  R04_ARRAY_SIZE(R04_ForwardCrawlSteps),  1U},
	{R04_BackwardCrawlSteps, R04_ARRAY_SIZE(R04_BackwardCrawlSteps), 1U},
	{R04_StrafeLeftSteps,    R04_ARRAY_SIZE(R04_StrafeLeftSteps),    1U},
	{R04_StrafeRightSteps,   R04_ARRAY_SIZE(R04_StrafeRightSteps),   1U},
	{R04_TurnInPlaceLeftSteps, R04_ARRAY_SIZE(R04_TurnInPlaceLeftSteps), 1U},
	{R04_TurnInPlaceLeftSteps, R04_ARRAY_SIZE(R04_TurnInPlaceLeftSteps), 1U}, /* 右转：同左表，目标在 StepTargetDeg 对 fb 取反 */
};

static R04_ActionRuntime R04_ActionState = {R04_ACTION_STAND, 0U, 0U};
/* 每次从遥控重新切入前进/后退时置 1：第 1 步用“全在中间”抬腿；完成该步后清 0 */
static uint8_t R04_CrawlUseMidStep1 = 0U;
static uint16_t R04_RemoteNeutralTick = 0U;
static uint8_t R04_RemoteArmed = 0U;
static uint8_t R04_ControlEnabled = 0U;
static uint8_t R04_ZeroPoseValid = 0U;
static float R04_ZeroPoseDeg[R04_MOTOR_COUNT] = {0.0f};
/* 偶数轴 m2,m4,m6,m8,m10,m12：使能瞬间记录的角度；失能时缓降终点 */
static float R04_EvenPreEnableDeg[6];
static uint8_t R04_EvenRampActive = 0U;
static uint16_t R04_EvenRampTick = 0U;
/* 失能缓降起点：当前反馈角；偶数轴终点为 R04_EvenPreEnableDeg，奇数轴终点同起点（仅竖直方向趴下） */
static float R04_DisableRampStartDeg[R04_MOTOR_COUNT];
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

/* 通道与动作成对：前后/左右转/左右平移 均 ± 对称；若实机左右反了，应交换对应通道或此处符号，勿只改单侧步表 */
static R04_ActionId R04_SelectActionFromRemote(void)
{
	float forward_cmd = Sbus_Value[2];
	float strafe_cmd = Sbus_Value[0];  /* 左负右正 → STRAFE_LEFT / STRAFE_RIGHT */
	float yaw_cmd = Sbus_Value[3];     /* 左正右负 → TURN_LEFT / TURN_RIGHT */

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

	if (forward_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_FORWARD_CRAWL;
	}

	if (forward_cmd < -R04_CMD_DEADBAND)
	{
		return R04_ACTION_BACKWARD_CRAWL;
	}

	if (yaw_cmd > R04_CMD_DEADBAND)
	{
		return R04_ACTION_TURN_LEFT;
	}

	if (yaw_cmd < -R04_CMD_DEADBAND)
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

	if ((R04_Abs(forward_cmd) < R04_CMD_DEADBAND) &&
	    (R04_Abs(strafe_cmd) < R04_CMD_DEADBAND) &&
	    (R04_Abs(yaw_cmd) < R04_CMD_DEADBAND))
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
	if (R04_ACTION_USES_CRAWL_MIT(action_id))
	{
		R04_CrawlUseMidStep1 = 1U;
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

/* 前摆相 ②a(step1)=三足 A 摆：fb 为 0,4,8；⑤a(step6)=三足 B 摆：fb 为 2,6,10 */
static uint8_t R04_CrawlSwingOscFbMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) != 0U)
	{
		return 0U;
	}
	if (step_idx == 1U)
	{
		return ((motor_idx == 0U) || (motor_idx == 4U) || (motor_idx == 8U)) ? 1U : 0U;
	}
	if (step_idx == 6U)
	{
		return ((motor_idx == 2U) || (motor_idx == 6U) || (motor_idx == 10U)) ? 1U : 0U;
	}
	return 0U;
}

/* ②a：摆动三足 A 的竖直轴 1,5,9；⑤a：摆动三足 B 的竖直轴 3,7,11 */
static uint8_t R04_CrawlSwingOscUdMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) == 0U)
	{
		return 0U;
	}
	if (step_idx == 1U)
	{
		return ((motor_idx == 1U) || (motor_idx == 5U) || (motor_idx == 9U)) ? 1U : 0U;
	}
	if (step_idx == 6U)
	{
		return ((motor_idx == 3U) || (motor_idx == 7U) || (motor_idx == 11U)) ? 1U : 0U;
	}
	return 0U;
}

/* ①：三足 A 抬起 ud 1,5,9；④：三足 B 抬起 ud 3,7,11 */
static uint8_t R04_CrawlLiftOscUdMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) == 0U)
	{
		return 0U;
	}
	if (step_idx == 0U)
	{
		return ((motor_idx == 1U) || (motor_idx == 5U) || (motor_idx == 9U)) ? 1U : 0U;
	}
	if (step_idx == 5U)
	{
		return ((motor_idx == 3U) || (motor_idx == 7U) || (motor_idx == 11U)) ? 1U : 0U;
	}
	return 0U;
}

/* ①/④ 抬腿相：撑地三足的竖直(ud)轴。step0 为 B 三足(ud 3,7,11)；step5 为 A 三足(ud 1,5,9) */
static uint8_t R04_ForwardTripodLiftStanceUdMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) == 0U)
	{
		return 0U;
	}
	if (step_idx == 0U)
	{
		return ((motor_idx == 3U) || (motor_idx == 7U) || (motor_idx == 11U)) ? 1U : 0U;
	}
	if (step_idx == 5U)
	{
		return ((motor_idx == 1U) || (motor_idx == 5U) || (motor_idx == 9U)) ? 1U : 0U;
	}
	return 0U;
}

/* 前摆平移 ②a/⑤a：单侧仅一足撑地时该足 ud 额外顶负载；step1 与 step6 共用 R04_FWD_SWING_SINGLE_SIDE_UD_* 倍率，左右对称 */
static uint8_t R04_ForwardTripodSwingSingleSideUdMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) == 0U)
	{
		return 0U;
	}
	if ((step_idx == 1U) && (motor_idx == R04_FWD_SWING_SINGLE_SIDE_UD_IDX_STEP1))
	{
		return 1U;
	}
	if ((step_idx == 6U) && (motor_idx == R04_FWD_SWING_SINGLE_SIDE_UD_IDX_STEP6))
	{
		return 1U;
	}
	return 0U;
}

/* ②a(step1)/⑤a(step6) 平移：撑地三足全部 ud（step1：B→3,7,11；step6：A→1,5,9） */
static uint8_t R04_ForwardTripodSwingTranslationAllStanceUdMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) == 0U)
	{
		return 0U;
	}
	if (step_idx == 1U)
	{
		return ((motor_idx == 3U) || (motor_idx == 7U) || (motor_idx == 11U)) ? 1U : 0U;
	}
	if (step_idx == 6U)
	{
		return ((motor_idx == 1U) || (motor_idx == 5U) || (motor_idx == 9U)) ? 1U : 0U;
	}
	return 0U;
}

/* ②b：摆腿落地时 B 组撑地 ud 3,7,11；⑤b：A 组撑地 ud 1,5,9（与抬腿相撑地三足同序） */
static uint8_t R04_ForwardTripodLandingStanceUdMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) == 0U)
	{
		return 0U;
	}
	if (step_idx == 2U)
	{
		return ((motor_idx == 3U) || (motor_idx == 7U) || (motor_idx == 11U)) ? 1U : 0U;
	}
	if (step_idx == 7U)
	{
		return ((motor_idx == 1U) || (motor_idx == 5U) || (motor_idx == 9U)) ? 1U : 0U;
	}
	return 0U;
}

/* ②b：A 组(1,3,5)落足；⑤b：B 组(2,4,6)落足 */
static uint8_t R04_CrawlLandingPlaceMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if (step_idx == 2U)
	{
		return ((motor_idx <= 1U) ||
		        ((motor_idx >= 4U) && (motor_idx <= 5U)) ||
		        ((motor_idx >= 8U) && (motor_idx <= 9U))) ? 1U : 0U;
	}
	if (step_idx == 7U)
	{
		return (((motor_idx >= 2U) && (motor_idx <= 3U)) ||
		        ((motor_idx >= 6U) && (motor_idx <= 7U)) ||
		        (motor_idx >= 10U)) ? 1U : 0U;
	}
	return 0U;
}

/* ③a③b、⑥a⑥b：六足均触地支撑，全部竖直(ud)轴用于抑「上顶」叠乘 */
static uint8_t R04_ForwardHexSupportUdMotor(uint8_t step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) == 0U)
	{
		return 0U;
	}
	return ((step_idx == 3U) || (step_idx == 4U) || (step_idx == 8U) || (step_idx == 9U)) ? 1U : 0U;
}

/* 原地转向步 3/7（MIT 映射 fwd=4、9）：六足同动水平平移；与前进 ③b/⑥b 同 MIT 序号但 hold 长，补与 ②a 末段同款的 fb 抑振 */
static uint8_t R04_TurnPlanarShiftFbOscMotor(uint8_t forward_step_idx, uint8_t motor_idx)
{
	if ((motor_idx & 1U) != 0U)
	{
		return 0U;
	}
	return ((forward_step_idx == 4U) || (forward_step_idx == 9U)) ? 1U : 0U;
}

/* 目标相对零位偏置（度）。左右一致性：右转=左转的水平系镜像（仅 fb 取反），与左转共用同一套力度与步表。 */
static float R04_StepTargetDeg(const R04_ActionStep *step, R04_ActionId action_id, uint8_t motor_idx)
{
	float target_deg = step->pos_deg[motor_idx];
	/* 右转：与左转共用步表，仅水平 fb（偶 motor_idx）取反；ud 不变。腿6不在此叠 M11，以免左/右转不对称 */
	if (action_id == R04_ACTION_TURN_RIGHT)
	{
		if ((motor_idx & 1U) == 0U)
		{
			target_deg = -target_deg;
		}
	}
	if (R04_ACTION_USES_CRAWL_MIT(action_id))
	{
#if R04_M11_FORWARD_FB_INVERT
		/* 转向已在右转分支对全部水平轴取反；此处再反腿6 会导致右转身腿6 与左转身不对称 */
		if ((motor_idx == 10U) && (action_id != R04_ACTION_TURN_LEFT) && (action_id != R04_ACTION_TURN_RIGHT))
		{
			target_deg = -target_deg;
		}
#endif
#if R04_M12_FORWARD_UD_INVERT
		if (motor_idx == 11U)
		{
			target_deg = -target_deg;
		}
#endif
		if (motor_idx == 10U)
		{
			target_deg += R04_LEG6_FB_TRIM_DEG;
		}
		if (motor_idx == 11U)
		{
			target_deg += R04_LEG6_UD_TRIM_DEG;
		}
	}
	if (R04_ZeroPoseValid != 0U)
	{
		target_deg += R04_ZeroPoseDeg[motor_idx];
	}
	return target_deg;
}

#if R04_FWD_SETTLE_WAIT
static uint8_t R04_ForwardMotorsNearCommand(const R04_ActionStep *step, R04_ActionId action_id)
{
	for (uint8_t idx = 0U; idx < R04_MOTOR_COUNT; idx++)
	{
		uint8_t mid = R04_MotorIds[idx];
		if (RobStride04_HasFeedback(mid) == 0U)
		{
			continue;
		}
		const Rs_Motor *motor = RobStride04_GetMotor(mid);
		if (motor == 0)
		{
			continue;
		}
		float cmd = R04_StepTargetDeg(step, action_id, idx);
		float fb = R04_RAD2DEG(motor->position);
		if (R04_Abs(fb - cmd) > R04_FWD_SETTLE_ERR_DEG)
		{
			return 0U;
		}
	}
	return 1U;
}
#endif /* R04_FWD_SETTLE_WAIT */

static void R04_ApplyActionStep(const R04_ActionStep *step, R04_ActionId action_id, uint8_t forward_step_idx)
{
	const float gait_user_mul = R04_GaitUserMulForAction(action_id, R04_GaitSpeedMulFromSbus());
	const uint32_t hold_ms_eff = R04_StepHoldMsScaled(step, gait_user_mul);
	const float speed_eff = R04_StepSpeedScaled(step->speed, gait_user_mul);

	for (uint8_t idx = 0U; idx < R04_MOTOR_COUNT; idx++)
	{
		float target_deg = R04_StepTargetDeg(step, action_id, idx);

		float kp = step->kp;
		float kd = step->kd;
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_ForwardTripodStanceMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_FWD_TRIPOD_STANCE_KP_MUL;
			kd *= R04_FWD_TRIPOD_STANCE_KD_MUL;
			if ((idx & 1U) != 0U)
			{
				kp *= R04_FWD_TRIPOD_STANCE_UD_KP_MUL;
				kd *= R04_FWD_TRIPOD_STANCE_UD_KD_MUL;
			}
			if (((forward_step_idx == 1U) || (forward_step_idx == 6U)) && ((idx & 1U) != 0U))
			{
				kp *= R04_FWD_SWING_STANCE_UD_KP_EXTRA;
				kd *= R04_FWD_SWING_STANCE_UD_KD_EXTRA;
			}
			if (R04_ForwardTripodSwingTranslationAllStanceUdMotor(forward_step_idx, idx) != 0U)
			{
				kp *= R04_FWD_SWING_TRANS_TRIPOD_UD_KP_MUL;
				kd *= R04_FWD_SWING_TRANS_TRIPOD_UD_KD_MUL;
			}
			if (R04_ForwardTripodSwingSingleSideUdMotor(forward_step_idx, idx) != 0U)
			{
				kp *= R04_FWD_SWING_SINGLE_SIDE_UD_KP_MUL;
				kd *= R04_FWD_SWING_SINGLE_SIDE_UD_KD_MUL;
			}
				if (R04_ForwardTripodLiftStanceUdMotor(forward_step_idx, idx) != 0U)
				{
					kp *= R04_FWD_TRIPOD_LIFT_STANCE_UD_KP_MUL;
					kd *= R04_FWD_TRIPOD_LIFT_STANCE_UD_KD_MUL;
					if (((uint32_t)R04_ActionState.step_tick * 2U) < hold_ms_eff)
					{
						kp *= R04_FWD_TRIPOD_LIFT_INSTANT_UD_KP_MUL;
						kd *= R04_FWD_TRIPOD_LIFT_INSTANT_UD_KD_MUL;
					}
				}
				if (R04_ForwardTripodLandingStanceUdMotor(forward_step_idx, idx) != 0U)
				{
					kp *= R04_FWD_LANDING_STANCE_UD_KP_MUL;
					kd *= R04_FWD_LANDING_STANCE_UD_KD_MUL;
					if (((uint32_t)R04_ActionState.step_tick * 2U) < hold_ms_eff)
					{
						kp *= R04_FWD_LANDING_STANCE_INSTANT_UD_KP_MUL;
						kd *= R04_FWD_LANDING_STANCE_INSTANT_UD_KD_MUL;
					}
				}
			}

		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_CrawlSwingOscFbMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_SWING_FB_OSC_KP_MUL;
			kd *= R04_SWING_FB_OSC_KD_MUL;
			if (((uint32_t)R04_ActionState.step_tick * 2U) >= hold_ms_eff)
			{
				kd *= R04_SWING_FB_OSC_END_KD_MUL;
			}
		}
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    ((action_id == R04_ACTION_TURN_LEFT) || (action_id == R04_ACTION_TURN_RIGHT)) &&
		    (R04_TurnPlanarShiftFbOscMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_SWING_FB_OSC_KP_MUL;
			kd *= R04_SWING_FB_OSC_KD_MUL;
			if (((uint32_t)R04_ActionState.step_tick * 2U) >= hold_ms_eff)
			{
				kd *= R04_SWING_FB_OSC_END_KD_MUL;
			}
		}
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_CrawlSwingOscUdMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_SWING_UD_OSC_KP_MUL;
			kd *= R04_SWING_UD_OSC_KD_MUL;
			if (((uint32_t)R04_ActionState.step_tick * 2U) >= hold_ms_eff)
			{
				kd *= R04_SWING_UD_OSC_END_KD_MUL;
			}
		}
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_CrawlLiftOscUdMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_LIFT_UD_OSC_KP_MUL;
			kd *= R04_LIFT_UD_OSC_KD_MUL;
			if (((uint32_t)R04_ActionState.step_tick * 2U) >= hold_ms_eff)
			{
				kd *= R04_LIFT_UD_OSC_END_KD_MUL;
			}
		}
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_CrawlLandingPlaceMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_FWD_LANDING_KP_MUL;
			kd *= R04_FWD_LANDING_KD_MUL;
			if ((idx & 1U) != 0U)
			{
				kd *= R04_FWD_LANDING_UD_KD_MUL;
				if (((uint32_t)R04_ActionState.step_tick * 2U) < hold_ms_eff)
				{
					kd *= R04_FWD_LANDING_TOUCH_UD_KD_MUL;
				}
			}
			if ((action_id == R04_ACTION_TURN_LEFT) || (action_id == R04_ACTION_TURN_RIGHT))
			{
				kp *= R04_TURN_LANDING_KP_EXTRA_MUL;
				kd *= R04_TURN_LANDING_KD_EXTRA_MUL;
				if ((idx & 1U) != 0U)
				{
					kd *= R04_TURN_LANDING_TOUCH_UD_KD_EXTRA_MUL;
				}
			}
		}
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_ForwardHexSupportUdMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_FWD_HEX_SUPPORT_UD_KP_MUL;
			kd *= R04_FWD_HEX_SUPPORT_UD_KD_MUL;
		}

		RobStride04_Set(R04_MotorIds[idx],
		                R04_DEG2RAD(target_deg),
		                speed_eff,
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
	for (uint8_t i = 0U; i < R04_MOTOR_COUNT; i++)
	{
		const Rs_Motor *motor = RobStride04_GetMotor(R04_MotorIds[i]);
		if (motor == 0)
		{
			R04_EvenRampActive = 0U;
			return;
		}
		R04_DisableRampStartDeg[i] = R04_RAD2DEG(motor->position);
	}
	R04_EvenRampActive = 1U;
	R04_EvenRampTick = 0U;
}

/* 目标角线性插值：alpha=t/T；偶数轴(ud) 从起点到使能前趴地角，奇数轴(fb) 起终点相同 */
static void R04_RunEvenRampDown(void)
{
	R04_EvenRampTick++;
	float alpha = (float)R04_EvenRampTick / (float)R04_EVEN_RAMP_DOWN_MS;
	if (alpha > 1.0f)
	{
		alpha = 1.0f;
	}
	const float rt = 0.0f;
	const float rs = R04_DISABLE_RAMP_SPEED;
	const float rkp = R04_DISABLE_RAMP_KP;
	const float rkd = R04_DISABLE_RAMP_KD;

	for (uint8_t i = 0U; i < R04_MOTOR_COUNT; i++)
	{
		float start_deg = R04_DisableRampStartDeg[i];
		float end_deg;
		if ((i & 1U) != 0U)
		{
			uint8_t ek = (uint8_t)(i >> 1U);
			end_deg = R04_EvenPreEnableDeg[ek];
		}
		else
		{
			end_deg = start_deg;
		}
		float target_deg = start_deg + (end_deg - start_deg) * alpha;
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
				R04_OnDisableStartRamp();
				R04_LastCtrlForRamp = 0U;
				/* 首帧立即下发缓降：先给力再 5s 内减到无力 */
				if (R04_EvenRampActive != 0U)
				{
					R04_RunEvenRampDown();
				}
				return;
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
	if (R04_ACTION_USES_CRAWL_MIT(selected_action) &&
	    (R04_ActionState.step_idx == 0U) && (R04_CrawlUseMidStep1 != 0U))
	{
		if ((selected_action == R04_ACTION_TURN_LEFT) || (selected_action == R04_ACTION_TURN_RIGHT))
		{
			step = &R04_TurnYawStep1_FromMid;
		}
		else
		{
			step = &R04_ForwardCrawlStep1_FromMid;
		}
	}
	{
		uint8_t fwd_idx = 0U;
		if (R04_ACTION_USES_CRAWL_MIT(selected_action))
		{
			fwd_idx = R04_ActionState.step_idx;
			if ((selected_action == R04_ACTION_TURN_LEFT) || (selected_action == R04_ACTION_TURN_RIGHT))
			{
				static const uint8_t kR04_TurnSimpleMitMap[8] = {0U, 1U, 2U, 4U, 5U, 6U, 7U, 9U};
				if (R04_ActionState.step_idx < (uint8_t)sizeof(kR04_TurnSimpleMitMap))
				{
					fwd_idx = kR04_TurnSimpleMitMap[R04_ActionState.step_idx];
				}
			}
		}
		R04_ApplyActionStep(step, selected_action, fwd_idx);
	}

	R04_ActionState.step_tick++;
	{
		const float gait_user_mul = R04_GaitUserMulForAction(selected_action, R04_GaitSpeedMulFromSbus());
		const uint32_t hold_scaled = R04_StepHoldMsScaled(step, gait_user_mul);

		if (R04_ActionState.step_tick >= hold_scaled)
		{
			uint8_t advance = 1U;
#if R04_FWD_SETTLE_WAIT
			if (R04_ACTION_USES_CRAWL_MIT(selected_action))
			{
				float se = (float)((uint32_t)R04_FWD_SETTLE_EXTRA_MS * (uint32_t)R04_GAIT_OBSERVE_SLOWDOWN_MUL) / gait_user_mul;
				if ((selected_action == R04_ACTION_TURN_LEFT) || (selected_action == R04_ACTION_TURN_RIGHT))
				{
					se += (float)((uint32_t)R04_TURN_SETTLE_EXTRA_MS * (uint32_t)R04_GAIT_OBSERVE_SLOWDOWN_MUL) / gait_user_mul;
				}
				uint32_t settle_extra = (se < 1.0f) ? 1U : (uint32_t)(se + 0.5f);
				uint32_t max_wait = hold_scaled + settle_extra;
				if ((uint32_t)R04_ActionState.step_tick < max_wait)
				{
					if (R04_ForwardMotorsNearCommand(step, selected_action) == 0U)
					{
						advance = 0U;
					}
				}
			}
#endif
			if (advance != 0U)
			{
				R04_ActionState.step_tick = 0U;
				if ((R04_ActionState.step_idx + 1U) < group->step_count)
				{
					if (R04_ACTION_USES_CRAWL_MIT(selected_action) &&
					    (R04_ActionState.step_idx == 0U) && (R04_CrawlUseMidStep1 != 0U))
					{
						R04_CrawlUseMidStep1 = 0U;
					}
					R04_ActionState.step_idx++;
				}
				else if (group->loop != 0U)
				{
					R04_ActionState.step_idx = 0U;
				}
			}
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
















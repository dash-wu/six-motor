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
 * 左右与力度对称（前进/后退共用同一套 step_idx 与倍率；后退仅水平目标取反，撑地腿组合不变）：
 * - ②a(step1) B 撑地：左列仅腿2，右列腿4+6 → 单腿加强 ud(3)；平移三足 ud 为 3,7,11。
 * - ⑤a(step6) A 撑地：左列腿1+3，右列仅腿5 → 单腿加强 ud(9)；平移三足 ud 为 1,5,9。
 * - ①(step0)/④(step5) 抬腿：对称地分别加强 B 三足 ud(3,7,11) 与 A 三足 ud(1,5,9)。
 * 单腿与三足平移的 kp/kd 宏各只有一对，step1/step6 共用，保证左右角色在整周期内力度对称。
 */
/* 前进/后退：step0~2、5~7 三足撑地加 kp/kd；step3~4、8~9 六足撑地时对 ud 另见 HEX_SUPPORT（抑上顶） */
#define R04_FWD_TRIPOD_STANCE_KP_MUL 1.72f
#define R04_FWD_TRIPOD_STANCE_KD_MUL 1.34f
/* 撑地腿竖直(ud)轴再略顶重力（奇数下标 m2,m4…）；与上行叠乘 */
#define R04_FWD_TRIPOD_STANCE_UD_KP_MUL 1.10f
#define R04_FWD_TRIPOD_STANCE_UD_KD_MUL 1.08f
/* 前摆相 ②a / ⑤a：对侧三足水平前摆、重心前移，仅再加强撑地腿「上下轴」刚度，减轻该段主体下坠 */
#define R04_FWD_SWING_STANCE_UD_KP_EXTRA 1.22f
#define R04_FWD_SWING_STANCE_UD_KD_EXTRA 1.10f
/* ②a/⑤a 前摆平移：三条撑地腿 ud 整体再略加（与 SWING_STANCE_UD_EXTRA 叠乘；不含抬腿①④） */
#define R04_FWD_SWING_TRANS_TRIPOD_UD_KP_MUL 1.10f
#define R04_FWD_SWING_TRANS_TRIPOD_UD_KD_MUL 1.07f
/* 同上平移相：左右仅单足一侧竖直轴再叠乘（与 R04_FWD_SWING_SINGLE_SIDE_UD_IDX_* 对应） */
#define R04_FWD_SWING_SINGLE_SIDE_UD_KP_MUL 1.27f
#define R04_FWD_SWING_SINGLE_SIDE_UD_KD_MUL 1.18f
#define R04_FWD_SWING_SINGLE_SIDE_UD_IDX_STEP1 3U /* 腿2 ud，左列单撑 */
#define R04_FWD_SWING_SINGLE_SIDE_UD_IDX_STEP6 9U /* 腿5 ud，右列单撑 */
/* ②b(step2)/⑤b(step7) 摆动足落地相：落足腿再叠乘；步前半段 ud 再抬 kd，抑砸地 */
#define R04_FWD_LANDING_KP_MUL          0.68f
#define R04_FWD_LANDING_KD_MUL          1.78f
#define R04_FWD_LANDING_UD_KD_MUL       1.52f
#define R04_FWD_LANDING_TOUCH_UD_KD_MUL 1.24f
/* 原地转向：在爬行落地叠乘之上再软化（②b/⑤b）；仅摆腿落地腿 */
#define R04_TURN_LANDING_KP_EXTRA_MUL       0.78f
#define R04_TURN_LANDING_KD_EXTRA_MUL       1.10f
#define R04_TURN_LANDING_TOUCH_UD_KD_EXTRA_MUL 1.12f
/* ①(step0)/④(step5) 抬腿相：对侧三足撑地 ud 加强，抑抬腿时机身下沉 */
#define R04_FWD_TRIPOD_LIFT_STANCE_UD_KP_MUL   1.28f
#define R04_FWD_TRIPOD_LIFT_STANCE_UD_KD_MUL   1.22f
#define R04_FWD_TRIPOD_LIFT_INSTANT_UD_KP_MUL  1.20f
#define R04_FWD_TRIPOD_LIFT_INSTANT_UD_KD_MUL  1.16f
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
/* 爬行水平摆幅（度）；③⑥ 插值中点为一半 */
#define R04_CRAWL_SWING_FB_DEG       (22.0f)
#define R04_CRAWL_PUSH_HALF_FB_DEG (11.0f)
/* 原地简化转向：小水平差动（度）、统一慢步长；无③④蹬伸相 */
#define R04_TURN_SIMPLE_STEP_MS      (900U)
#define R04_TURN_SIMPLE_YAW_FB_DEG   (10.0f)
#define R04_TURN_SETTLE_EXTRA_MS     (90U) /* 转向专用到位等待上限增量，与 FWD_SETTLE 叠用 */
/* 摇杆回中后等待多久才允许步态；0=使能后立即响应（仅 Sbus[4] 与回连有效时仍需回中） */
#define R04_REMOTE_ARM_MS 0U
#define R04_CONTROL_DISABLE_TH (-0.5f)
#define R04_CONTROL_ENABLE_TH  (0.5f)
/* Sbus[4] 使能后偶数轴相对使能前再“下压”撑起机身（度），符号可按实机反向改 */
/* 使能瞬间偶数轴相对“趴地”姿态多压下的角度；越大机身抬得越高，过小撑不起来，可实机微调（仅改高度，不改 MIT 力度） */
#define R04_BODY_LIFT_EVEN_DEG (23.0f)
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
/* 取反与零位之后叠加（度），装配/表意微调；一般保持 0 */
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
	 * hold：再减半，步频再约×2；③ 仅 19ms 极易跟不上，实机可单独加长
	 */

	/* ① 三足 A(1、3、5) 抬起；B(2、4、6) 不动 */
	R04_STEP(26U, 1.0f, 0.0f, 44.0f, 0.58f,
	        -18.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-18.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, 18.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②a A 完全抬起后向前摆；B 不动（略低 kp、略高 kd 基底，配合代码减晃） */
	R04_STEP(70U, 0.0f, 0.0f, 20.0f, 0.82f,
	         R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②b A 放下（基底略软+略阻尼；落足腿另在 Apply 中叠乘） */
	R04_STEP(66U, 0.36f, 0.0f, 11.0f, 1.05f,
	         R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,
	          0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f),
	/* ③a 全体后蹬前半：六腿水平(fb)一起从 ②b 向「A 中、B 后」插值 */
	R04_STEP(19U, 1.0f, 0.0f, 42.0f, 0.58f,
	         R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,-R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,
	         R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,-R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, -R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f),
	/* ③b 全体后蹬末：腿1、3、5 水平≈中位；腿2、4、6 水平≈后侧（机体已前移） */
	R04_STEP(19U, 1.0f, 0.0f, 42.0f, 0.58f,
	          0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,
	         R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f, -R04_CRAWL_SWING_FB_DEG,  0.0f),
	/* ④ B 抬起；A 保持 ③b（腿6 水平保持③b 后侧 -SWING_FB，与腿4 保持 m7 同理，只抬竖直） */
	R04_STEP(26U, 1.0f, 0.0f, 44.0f, 0.58f,
	          0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	         R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤a B 前摆；A 不动 */
	R04_STEP(70U, 0.0f, 0.0f, 20.0f, 0.82f,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	        -R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤b B 落地 */
	R04_STEP(66U, 0.36f, 0.0f, 11.0f, 1.05f,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,
	        -R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f),
	/* ⑥a 全体后蹬前半：从 ⑤b 向「B 中、A 后」插值 */
	R04_STEP(19U, 1.0f, 0.0f, 42.0f, 0.58f,
	        -R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,-R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,
	        -R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f),
	/* ⑥b 全体后蹬末：腿2、4、6 水平≈中位；腿1、3、5 水平≈后侧 → 接下一循环 ① */
	R04_STEP(38U, 1.0f, 0.0f, 42.0f, 0.58f,
	        -R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f,
	         0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f),
};

/* 与上表步①同参数，但三足 A 水平角为 0；首次切入前进/后退共用这一步 */
static const R04_ActionStep R04_ForwardCrawlStep1_FromMid =
	R04_STEP(26U, 1.0f, 0.0f, 44.0f, 0.58f,
	          0.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,  0.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f);

/* 转向切入步①：几何同 FromMid，hold 与简化转向步① 一致 */
static const R04_ActionStep R04_TurnYawStep1_FromMid =
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.42f, 0.0f, 40.0f, 0.56f,
	          0.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,  0.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f);

/* 后退爬行：与 R04_ForwardCrawlSteps 同结构、同 hold/kp；水平轴(fb,奇数下标)取反，竖直(ud)不变 → 机体沿反方向爬行 */
static const R04_ActionStep R04_BackwardCrawlSteps[] =
{
	/*  m1     m2     m3     m4     m5     m6     m7     m8     m9    m10    m11    m12  */
	/* ① */
	R04_STEP(26U, 1.0f, 0.0f, 44.0f, 0.58f,
	         18.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, 18.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f,-18.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②a */
	R04_STEP(70U, 0.0f, 0.0f, 20.0f, 0.82f,
	        -R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* ②b */
	R04_STEP(66U, 0.36f, 0.0f, 11.0f, 1.05f,
	        -R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f),
	/* ③a */
	R04_STEP(19U, 1.0f, 0.0f, 42.0f, 0.58f,
	        -R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,-R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,
	        -R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f),
	/* ③b */
	R04_STEP(19U, 1.0f, 0.0f, 42.0f, 0.58f,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,
	        -R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f),
	/* ④ */
	R04_STEP(26U, 1.0f, 0.0f, 44.0f, 0.58f,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	        -R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤a */
	R04_STEP(70U, 0.0f, 0.0f, 20.0f, 0.82f,
	          0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	         R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* ⑤b */
	R04_STEP(66U, 0.36f, 0.0f, 11.0f, 1.05f,
	          0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,
	         R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f),
	/* ⑥a */
	R04_STEP(19U, 1.0f, 0.0f, 42.0f, 0.58f,
	         R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,-R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f, R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,
	         R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,-R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f,-R04_CRAWL_PUSH_HALF_FB_DEG,  0.0f),
	/* ⑥b */
	R04_STEP(38U, 1.0f, 0.0f, 42.0f, 0.58f,
	         R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,
	          0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f),
};

/*
 * 原地左转(CCW)：7 步闭环，无③⑥蹬伸相；慢速统一 hold；右转共用本表，R04_StepTargetDeg 对 fb 取反。
 * 逻辑步序 0..6 对应爬行 MIT 辅助下标 0,1,2,5,6,7,9（见 MS_Task 中 fwd_idx 映射）。
 */
static const R04_ActionStep R04_TurnInPlaceLeftSteps[] =
{
	/* 0 → MIT 0：① 抬 A */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.42f, 0.0f, 40.0f, 0.56f,
	        -18.0f, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-18.0f, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, 18.0f, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* 1 → MIT 1：②a A 小偏航摆腿 */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.0f, 0.0f, 18.0f, 0.82f,
	         R04_TURN_SIMPLE_YAW_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,-R04_TURN_SIMPLE_YAW_FB_DEG, R04_CRAWL_LIFT_UD_DEG,
	          0.0f,  0.0f, R04_TURN_SIMPLE_YAW_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f),
	/* 2 → MIT 2：②b A 落足 */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.30f, 0.0f, 9.5f, 1.10f,
	         R04_TURN_SIMPLE_YAW_FB_DEG,  0.0f,  0.0f,  0.0f,-R04_TURN_SIMPLE_YAW_FB_DEG,  0.0f,
	          0.0f,  0.0f, R04_TURN_SIMPLE_YAW_FB_DEG,  0.0f,  0.0f,  0.0f),
	/* 3 → MIT 5：④ 抬 B；A 保持②b 偏航支撑（避免跳变到爬行③b） */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.42f, 0.0f, 40.0f, 0.56f,
	         R04_TURN_SIMPLE_YAW_FB_DEG,  0.0f,-R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,-R04_TURN_SIMPLE_YAW_FB_DEG,  0.0f,
	         R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG, R04_TURN_SIMPLE_YAW_FB_DEG,  0.0f,-R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* 4 → MIT 6：⑤a B 前摆 */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.0f, 0.0f, 18.0f, 0.82f,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f,
	        -R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG, -R04_CRAWL_LIFT_UD_DEG),
	/* 5 → MIT 7：⑤b B 落地 */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.30f, 0.0f, 9.5f, 1.10f,
	          0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,
	        -R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f),
	/* 6 → MIT 9：⑥b 末态，接下一圈 ① */
	R04_STEP(R04_TURN_SIMPLE_STEP_MS, 0.42f, 0.0f, 38.0f, 0.58f,
	        -R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f,-R04_CRAWL_SWING_FB_DEG,  0.0f,
	         0.0f,  0.0f, R04_CRAWL_SWING_FB_DEG,  0.0f,  0.0f,  0.0f),
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

static R04_ActionId R04_SelectActionFromRemote(void)
{
	float forward_cmd = Sbus_Value[2];
	/* [0]：左右平移 */
	float strafe_cmd = Sbus_Value[0];
	/* [3]：原地转向，>0 左转、<0 右转 */
	float yaw_cmd = Sbus_Value[3];

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

static float R04_StepTargetDeg(const R04_ActionStep *step, R04_ActionId action_id, uint8_t motor_idx)
{
	float target_deg = step->pos_deg[motor_idx];
	/* 右转：与左转共用步表，仅水平轴(fb，偶数 motor_idx)取反 → CW；避免与腿6 fb 取反宏叠加产生左右不对称 */
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
		if (motor_idx == 10U)
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
	for (uint8_t idx = 0; idx < R04_MOTOR_COUNT; idx++)
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
				if (((uint32_t)R04_ActionState.step_tick * 2U) < (uint32_t)step->hold_ms)
				{
					kp *= R04_FWD_TRIPOD_LIFT_INSTANT_UD_KP_MUL;
					kd *= R04_FWD_TRIPOD_LIFT_INSTANT_UD_KD_MUL;
				}
			}
		}

		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_CrawlSwingOscFbMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_SWING_FB_OSC_KP_MUL;
			kd *= R04_SWING_FB_OSC_KD_MUL;
			if (((uint32_t)R04_ActionState.step_tick * 2U) >= (uint32_t)step->hold_ms)
			{
				kd *= R04_SWING_FB_OSC_END_KD_MUL;
			}
		}
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_CrawlSwingOscUdMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_SWING_UD_OSC_KP_MUL;
			kd *= R04_SWING_UD_OSC_KD_MUL;
			if (((uint32_t)R04_ActionState.step_tick * 2U) >= (uint32_t)step->hold_ms)
			{
				kd *= R04_SWING_UD_OSC_END_KD_MUL;
			}
		}
		if (R04_ACTION_USES_CRAWL_MIT(action_id) &&
		    (R04_CrawlLiftOscUdMotor(forward_step_idx, idx) != 0U))
		{
			kp *= R04_LIFT_UD_OSC_KP_MUL;
			kd *= R04_LIFT_UD_OSC_KD_MUL;
			if (((uint32_t)R04_ActionState.step_tick * 2U) >= (uint32_t)step->hold_ms)
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
				if (((uint32_t)R04_ActionState.step_tick * 2U) < (uint32_t)step->hold_ms)
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
				static const uint8_t kR04_TurnSimpleMitMap[7] = {0U, 1U, 2U, 5U, 6U, 7U, 9U};
				if (R04_ActionState.step_idx < (uint8_t)sizeof(kR04_TurnSimpleMitMap))
				{
					fwd_idx = kR04_TurnSimpleMitMap[R04_ActionState.step_idx];
				}
			}
		}
		R04_ApplyActionStep(step, selected_action, fwd_idx);
	}

	R04_ActionState.step_tick++;
	if (R04_ActionState.step_tick >= step->hold_ms)
	{
		uint8_t advance = 1U;
#if R04_FWD_SETTLE_WAIT
		if (R04_ACTION_USES_CRAWL_MIT(selected_action))
		{
			uint32_t settle_extra = (uint32_t)R04_FWD_SETTLE_EXTRA_MS;
			if ((selected_action == R04_ACTION_TURN_LEFT) || (selected_action == R04_ACTION_TURN_RIGHT))
			{
				settle_extra += (uint32_t)R04_TURN_SETTLE_EXTRA_MS;
			}
			uint32_t max_wait = (uint32_t)step->hold_ms + settle_extra;
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
















#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace diffnav {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 2.0f * kPi;
inline float clampf(float v,float lo,float hi){return std::max(lo,std::min(v,hi));}
inline float wrapAngle(float a){while(a>kPi)a-=kTwoPi;while(a<=-kPi)a+=kTwoPi;return a;}
inline float signf(float v){return v>0?1.0f:(v<0?-1.0f:0.0f);}
inline float hypot2(float x,float y){return std::sqrt(x*x+y*y);}

struct Pose2D{float x_mm=0,y_mm=0,theta_rad=0,theta_unwrapped_rad=0;};
struct WheelSpeeds{float right_mm_s=0,left_mm_s=0;};
struct BodyVelocity{float linear_mm_s=0,angular_rad_s=0;};
struct OdometryState{Pose2D pose{};WheelSpeeds wheel_speed{};BodyVelocity body_velocity{};int64_t right_ticks=0,left_ticks=0;};
enum class MotionMode:uint8_t{IDLE,VELOCITY,ALIGN,DRIVE_LINE,FOLLOW_BEZIER,ROTATE,FINAL_ORIENT,EMERGENCY_STOP,FAULT};
enum class MotionResult:uint8_t{IDLE,RUNNING,SUCCEEDED,CANCELLED,FAULTED};
enum class FaultCode:uint8_t{NONE,ENCODER_INIT,CONTROL_TIMING,STALL_RIGHT,STALL_LEFT,INVALID_COMMAND};
struct MotionStatus{MotionMode mode=MotionMode::IDLE;MotionResult result=MotionResult::IDLE;FaultCode fault=FaultCode::NONE;float progress_mm=0,remaining_mm=0,cross_track_mm=0,heading_error_rad=0;};
enum class NavCommandType:uint8_t{NONE,STOP,EMERGENCY_STOP,CLEAR_EMERGENCY,VELOCITY,GO_TO,MOVE_DISTANCE,ROTATE_RELATIVE,ORIENT_ABSOLUTE,SET_POSE,BEZIER};
struct NavCommand{NavCommandType type=NavCommandType::NONE;float p0=0,p1=0,p2=0,p3=0,p4=0,p5=0,p6=0,p7=0;int8_t direction=1;bool use_final_heading=false;};
struct Point2D{float x=0,y=0;};

struct OdometryConfig{float right_wheel_diameter_mm=34.73184056f,left_wheel_diameter_mm=34.69274232f,track_width_mm=110.37343379f;int32_t counts_per_revolution=2800;float speed_filter_tau_s=0.030f;};
struct NavigatorConfig{
 float track_width_mm=110.37343379f,max_wheel_speed_mm_s=750;
 float line_accel_mm_s2=900,line_decel_mm_s2=1200,line_min_speed_mm_s=55,default_line_speed_mm_s=450;
 float bezier_accel_mm_s2=750,bezier_decel_mm_s2=1000,bezier_min_speed_mm_s=50,default_bezier_speed_mm_s=350;
 float rotate_wheel_speed_mm_s=260,rotate_accel_mm_s2=700,rotate_decel_mm_s2=900,rotate_min_wheel_speed_mm_s=35;
 float heading_kp=4.5f,cross_track_gain_s_inv=3.0f,stanley_gain=2.4f,stanley_softening_mm_s=90,max_angular_rate_rad_s=5.5f;
 float final_position_kp_s_inv=3.0f,final_speed_limit_mm_s=120,final_heading_kp=5.0f;
 float position_tolerance_mm=2,cross_track_tolerance_mm=3,heading_tolerance_rad=0.0174532925f,align_tolerance_rad=0.0349065850f,stopped_speed_tolerance_mm_s=12;uint16_t settle_cycles=8;
};
struct WheelControllerConfig{float kp=.80f,ki=3.0f,kd=0,derivative_tau_s=.025f,antiwindup_gain_s_inv=8,integral_limit_pwm=260,pwm_limit=1023,static_feedforward_pwm=90,velocity_feedforward_pwm_per_mm_s=1.20f,pwm_slew_per_s=9000;};

inline WheelSpeeds bodyToWheels(float linear,float angular,float track){float h=.5f*track;return{linear+angular*h,linear-angular*h};}
inline WheelSpeeds limitWheelSpeeds(WheelSpeeds t,float m){float p=std::max(std::fabs(t.right_mm_s),std::fabs(t.left_mm_s));if(p>m&&p>1e-6f){float s=m/p;t.right_mm_s*=s;t.left_mm_s*=s;}return t;}
inline float profileSpeed(float progress,float remaining,float cruise,float accel,float decel,float minv){if(remaining<=0||cruise<=0)return 0;progress=std::max(0.0f,progress);remaining=std::max(0.0f,remaining);accel=std::max(1.0f,accel);decel=std::max(1.0f,decel);minv=std::max(0.0f,minv);float va=std::sqrt(minv*minv+2*accel*progress),vb=std::sqrt(2*decel*remaining);float v=std::min(cruise,std::min(va,vb));if(remaining>1)v=std::max(v,std::min(minv,vb));return std::max(0.0f,v);}
inline int32_t extendLegacyPcntDelta(int16_t previous,int16_t current,int32_t limit=30000){int32_t d=(int32_t)current-(int32_t)previous,h=limit/2;if(d<-h)d+=limit;else if(d>h)d-=limit;return d;}

class WheelSpeedController{
 public: explicit WheelSpeedController(WheelControllerConfig c={}):c_(c){};void reset(float out=0){i_=d_=prev_m_=0;prev_out_=out;init_=false;}float update(float target,float measured,float dt){if(!(dt>0)||dt>.1f)return prev_out_;if(std::fabs(target)<.5f){reset();return 0;}float e=target-measured;if(!init_){prev_m_=measured;init_=true;}float raw=-(measured-prev_m_)/dt,a=dt/(c_.derivative_tau_s+dt);d_+=a*(raw-d_);prev_m_=measured;float ff=signf(target)*(c_.static_feedforward_pwm+c_.velocity_feedforward_pwm_per_mm_s*std::fabs(target));i_+=c_.ki*e*dt;i_=clampf(i_,-c_.integral_limit_pwm,c_.integral_limit_pwm);float u=ff+c_.kp*e+i_+c_.kd*d_,sat=clampf(u,-c_.pwm_limit,c_.pwm_limit);i_+=c_.antiwindup_gain_s_inv*(sat-u)*dt;i_=clampf(i_,-c_.integral_limit_pwm,c_.integral_limit_pwm);float md=c_.pwm_slew_per_s*dt;sat=clampf(sat,prev_out_-md,prev_out_+md);prev_out_=sat;return sat;}
 private:WheelControllerConfig c_{};float i_=0,d_=0,prev_m_=0,prev_out_=0;bool init_=false;
};

class DifferentialOdometry{
 public: explicit DifferentialOdometry(OdometryConfig c={}):c_(c){};void reset(const Pose2D&p,int64_t r,int64_t l);void setPose(const Pose2D&p);void update(int64_t r,int64_t l,float dt);const OdometryState&state()const{return s_;}float rightTicksToDistance(int64_t t)const;float leftTicksToDistance(int64_t t)const;int64_t rightDistanceToTicks(float d)const;int64_t leftDistanceToTicks(float d)const;
 private:OdometryConfig c_{};OdometryState s_{};int64_t pr_=0,pl_=0;bool init_=false;
};

class CubicBezierPath{
 public:static constexpr int N=64;void set(Point2D p0,Point2D p1,Point2D p2,Point2D p3);bool valid()const{return valid_;}Point2D point(float t)const;Point2D derivative(float t)const;Point2D secondDerivative(float t)const;float heading(float t)const;float curvature(float t)const;float signedCrossTrack(float x,float y,float t)const;float project(float x,float y,float hint)const;float arcLengthAt(float t)const;float totalLength()const{return arc_[N];}
 private:Point2D p0_{},p1_{},p2_{},p3_{};std::array<float,N+1>arc_{};bool valid_=false;
};

class Navigator{
 public:explicit Navigator(NavigatorConfig c={});void stop();void emergencyStop(FaultCode f=FaultCode::NONE);void clearEmergency();void commandVelocity(float l,float a);void commandMoveDistance(const Pose2D&p,float d,float v);void commandRotateRelative(const Pose2D&p,float a,float v=0);void commandOrientAbsolute(const Pose2D&p,float a,float v=0);void commandGoTo(const Pose2D&p,float x,float y,float v,int dir,bool final=false,float phi=0);void commandBezier(const Pose2D&p,Point2D p1,Point2D p2,Point2D p3,float v,int dir);WheelSpeeds update(const OdometryState&o,float dt);const MotionStatus&status()const{return st_;}MotionMode mode()const{return st_.mode;}bool busy()const{return st_.result==MotionResult::RUNNING;}
 private:void beginLine(const Pose2D&p,Point2D target,float v,int dir);WheelSpeeds line(const OdometryState&o);WheelSpeeds rotate(const OdometryState&o);WheelSpeeds bezier(const OdometryState&o);WheelSpeeds align(const OdometryState&o);WheelSpeeds body(float l,float a)const;bool stopped(const OdometryState&o)const;void succeeded();NavigatorConfig c_{};MotionStatus st_{};float vl_t_=0,va_t_=0,vl_=0,va_=0;Point2D ls_{},lt_{};float lux_=1,luy_=0,ll_=0,lh_=0,lv_=0;int ldir_=1;float target_phi_u_=0,rot_len_=0,rot_v_=0;Point2D gt_{};float gv_=0;int gdir_=1;bool goto_=false,gfinal_=false;float gphi_=0;CubicBezierPath bp_{};float bt_=0,bv_=0;int bdir_=1;bool b_after_align_=false;uint16_t settle_=0;
};
}

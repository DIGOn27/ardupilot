#include "Plane.h"
#include <vector>
#include <tuple>


/*
set wing angle output
*/
void Plane::set_tilt_wing_out()
{
    //Automated tilt wing control
    if(plane.control_mode == &plane.mode_ttwstabilize || plane.control_mode == &plane.mode_ttwhover || plane.control_mode == &plane.mode_ttwloiter){
        float front_wing_out, back_wing_out;
        std::tie(front_wing_out, back_wing_out)= wing_tilt_control();
        SRV_Channels::set_output_scaled(SRV_Channel::k_front_wing_tilt, front_wing_out);
        SRV_Channels::set_output_scaled(SRV_Channel::k_back_wing_tilt, back_wing_out);

        SRV_Channels::set_slew_rate(SRV_Channel::k_front_wing_tilt, g.flap_slewrate, 9000, G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_back_wing_tilt, g.flap_slewrate, 9000, G_Dt);
        return;
    }

    //Yaw control with tilt wings test
    else if(plane.control_mode == &plane.mode_ttwyaw){
        //normal manual tilt wing control
        float front_wing_tilt_percent = 0;
        float back_wing_tilt_percent = 0;

        front_wing_tilt_percent = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WING_TILT)->norm_input_ignore_trim();
        back_wing_tilt_percent = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WING_TILT)->norm_input_ignore_trim();

        float front_wing_out  = constrain_float(front_wing_tilt_percent * 4500, -4500, 4500);
        float back_wing_out = constrain_float(back_wing_tilt_percent * 4500, -4500, 4500);
        
        SRV_Channels::set_output_scaled(SRV_Channel::k_front_wing_tilt, front_wing_out);
        SRV_Channels::set_output_scaled(SRV_Channel::k_back_wing_tilt, back_wing_out);

        SRV_Channels::set_slew_rate(SRV_Channel::k_front_wing_tilt, g.flap_slewrate, 9000, G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_back_wing_tilt, g.flap_slewrate, 9000, G_Dt);


        //yaw input 
        //ADD parameter to define max wing deflection for yaw
        float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder) /45 *g.tilt_wing_yaw_max; //scale rudder input to max wing deflection for yaw

        float front_wing_percent = SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_front_wing_tilt);
        float back_wing_percent = SRV_Channels::get_slew_limited_output_scaled(SRV_Channel::k_back_wing_tilt);


        front_wing_out  = constrain_float(rudder + front_wing_percent * 45, -4500, 4500);
        back_wing_out = constrain_float(-rudder + back_wing_percent * 45, -4500, 4500);

        SRV_Channels::set_output_scaled(SRV_Channel::k_front_wing_tilt, front_wing_out);
        SRV_Channels::set_output_scaled(SRV_Channel::k_back_wing_tilt, back_wing_out);


        float pitch_d;
        pitch_d = degrees(quadplane.ahrs_view->pitch);
        
        
        AP::logger().Write("TTW", "TimeUS,Pitch_d,frontW_d,backW_d", "Qfff",
                                        AP_HAL::micros64(),
                                        pitch_d,
                                        front_wing_out/100,
                                        back_wing_out/100);
    }
    
    // Manual tilt wing control
    else {
        
        //RC_Channel *channel_tiltwing = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WING_TILT);
        
        float front_wing_tilt_percent = 0;
        float back_wing_tilt_percent = 0;

        front_wing_tilt_percent = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WING_TILT)->norm_input_ignore_trim();
        back_wing_tilt_percent = rc().find_channel_for_option(RC_Channel::AUX_FUNC::WING_TILT)->norm_input_ignore_trim();

        float front_wing_out  = constrain_float(front_wing_tilt_percent * 4500, -4500, 4500);
        float back_wing_out = constrain_float(back_wing_tilt_percent * 4500, -4500, 4500);
        
        SRV_Channels::set_output_scaled(SRV_Channel::k_front_wing_tilt, front_wing_out);
        SRV_Channels::set_output_scaled(SRV_Channel::k_back_wing_tilt, back_wing_out);

        SRV_Channels::set_slew_rate(SRV_Channel::k_front_wing_tilt, g.flap_slewrate, 9000, G_Dt);
        SRV_Channels::set_slew_rate(SRV_Channel::k_back_wing_tilt, g.flap_slewrate, 9000, G_Dt);


        float pitch_d;
        pitch_d = degrees(quadplane.ahrs_view->pitch);
        

        AP::logger().Write("TTW", "TimeUS,Pitch_d,frontW_d,backW_d", "Qfff",
                                        AP_HAL::micros64(),
                                        pitch_d,
                                        front_wing_out/100,
                                        back_wing_out/100);
    }
    

}

/*
wing tilt control
*/
std::tuple<float, float> Plane::wing_tilt_control()
{
    // pitch - wing angle points
    std::vector<float> X = {g.tilt_wing_x1, g.tilt_wing_x2, g.tilt_wing_x3, g.tilt_wing_x4, g.tilt_wing_x5, g.tilt_wing_x6, g.tilt_wing_x7, g.tilt_wing_x8, g.tilt_wing_x9, g.tilt_wing_x10, g.tilt_wing_x11, g.tilt_wing_x12, g.tilt_wing_x13};
    std::vector<float> Y = {g.tilt_wing_y1, g.tilt_wing_y2, g.tilt_wing_y3, g.tilt_wing_y4, g.tilt_wing_y5, g.tilt_wing_y6, g.tilt_wing_y7, g.tilt_wing_y8, g.tilt_wing_y9, g.tilt_wing_y10, g.tilt_wing_y11, g.tilt_wing_y12, g.tilt_wing_y13};

    // control logic here
    float pitch_rad, pitch_d, front_wing_out, back_wing_out;
    pitch_rad = quadplane.ahrs_view->pitch;
    pitch_d = degrees(pitch_rad);

    pitch_d = constrain_float(pitch_d, -quadplane.aparm.angle_max, quadplane.aparm.angle_max); //constrain pitch to be within max angle limit

    if (pitch_d < 0.0) { //if negative pitch, use positive to get wing angle from table, then negate output to get correct direction of tilt
        float neg_pitch_d = -pitch_d;
        int L = 0;
        int R = X.size() - 1;
        int M = (L + R)/2;


        while (L <= R) {    //binary search to find correct table entries for interpolation
            if (X[M] < neg_pitch_d) {
                L = M + 1;
            } else {
                R = M - 1;
            }
            M = (L + R)/2;
        }
        //interpolate between the two table entries to get wing angle, then negate to get correct direction of tilt (value in degrees)
        front_wing_out = -(Y[R] + (Y[L] - Y[R]) * (neg_pitch_d - X[R]) / (X[L] - X[R]));
        back_wing_out = -(Y[R] + (Y[L] - Y[R]) * (neg_pitch_d - X[R]) / (X[L] - X[R]));
    }

        else { //if positive pitch, use table directly
            int L = 0;
	        int R = X.size() - 1;
	        int M = (L + R)/2;


            while (L <= R) {    //binary search to find correct table entries for interpolation
                if (X[M] < pitch_d) {
                    L = M + 1;
                } else {
                    R = M - 1;
                }
                M = (L + R)/2;
            }
            //interpolate between the two table entries to get wing angle (value in degrees)
            front_wing_out = Y[R] + (Y[L] - Y[R]) * (pitch_d - X[R]) / (X[L] - X[R]);
            back_wing_out = Y[R] + (Y[L] - Y[R]) * (pitch_d - X[R]) / (X[L] - X[R]);
        }
    



    //front_wing_out = pitch_cd;
    //min_angle = -4500; //-45 degrees in centidegrees
    //max_angle = 4500;  //45 degrees in centidegrees
    //front_wing_out = constrain_float(front_wing_out, min_angle, max_angle);
    




AP::logger().Write("TTW", "TimeUS,Pitch_d,frontW_d,backW_d", "Qfff",
                                        AP_HAL::micros64(),
                                        pitch_d,
                                        front_wing_out,
                                        back_wing_out);




    return std::make_tuple(front_wing_out*100, back_wing_out*100); //values in centidegrees
}
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TelemetrySystem
{
    public class Logs
    {
        public string filePath;
        public string fileName;
        public string file;
        

        public Logs(string path)
        {
            DateTime dt = DateTime.Now;
            string header = string.Empty;
            string sistemsaati;
            sistemsaati = DateTime.Now.ToLongTimeString();
            filePath = path;
            fileName = DateTime.Now.ToString("yyyy_MM_dd") + ".txt"; ;
            //fileName = DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss") + ".txt"; ;
            file = filePath + fileName;

            if (File.Exists(file))
            {
                
            }
            else
            {
                header = "Sistem Saati" + "\t" + "\t" + "Uçuş Süresi " + "\t" + "\t" + "Pitch_Angle " + "\t" + "\t" + "Roll_Angle " + "\t" + "\t" + "Yaw_Angle " + "\t" + "\t"
                     + "Pitch_Error " + "\t" + "\t" + "Roll_Error " + "\t" + "\t" + "Yaw_Error " + "\t"
                     + "\t" + "Motor1_PWM " + "\t" + "\t" + "Motor2_PWM " + "\t" + "\t" + "Motor3_PWM " + "\t" + "\t" + "Motor4_PWM "
                     + "\t" + "\t" + "RollPitch_Kp  " + "\t" + "\t" + "RollPitch_Ki " + "\t" + "\t" + "RollPitch_Kd " + "\t" + "\t" + "Yaw_Kp " + "\t" + "\t" + "Yaw_Ki " + "\t" + "\t" + "Yaw_Kd "
                     + "\t" + "\t" + "Altitude_Kp " + "\t" + "\t" + "Altitude_Ki " + "\t" + "\t" + "Altitude_Kd " + "\t"
                     + "\t" + "Barometer_Altitude " + "\t" + "\t" + "Lattitude " + "\t" + "\t" + "Longtitude ";

                StreamWriter sw = new StreamWriter(file, append: true);

                sw.WriteLine(header);

                sw.Close();
            }

            /*
            header = "Sistem Saati" + "\t" + "\t" + "Uçuş Süresi " + "\t" + "\t" + "Pitch_Angle " + "\t" + "\t" + "Roll_Angle " + "\t" + "\t" + "Yaw_Angle " + "\t" + "\t"
                     + "Pitch_Error " + "\t" + "\t" + "Roll_Error " + "\t" + "\t" + "Yaw_Error " + "\t"
                     + "\t" + "Motor1_PWM " + "\t" + "\t" + "Motor2_PWM " + "\t" + "\t" + "Motor3_PWM " + "\t" + "\t" + "Motor4_PWM "
                     + "\t" + "\t" + "RollPitch_Kp  " + "\t" + "\t" + "RollPitch_Ki " + "\t" + "\t" + "RollPitch_Kd " + "\t" + "\t" + "Yaw_Kp " + "\t" + "\t" + "Yaw_Ki " + "\t" + "\t" + "Yaw_Kd "
                     + "\t" + "\t" + "Altitude_Kp " + "\t" + "\t" + "Altitude_Ki " + "\t" + "\t" + "Altitude_Kd " + "\t"
                     + "\t" + "Barometer_Altitude " + "\t" + "\t" + "Lattitude " + "\t" + "\t"  + "Longtitude ";

            StreamWriter sw = new StreamWriter(file, append: true);

            sw.WriteLine(header);

            sw.Close();
            */
        }

        


        public void WriteLine(string logtime, object saattxt)
        {
            string line
                = saattxt + "\t" + "\t" + logtime + "\t" + "\t" + Datas.Pitch_angleX.ToString("0.00") + "\t"
                + "\t" + "\t" + Datas.Roll_angleY.ToString("0.00") + "\t"
                + "\t" + "\t" + Datas.Yaw_angleZ.ToString("0.00") + "\t"
                + "\t" + "\t" + Datas.Pitch_error_angleX.ToString("0.00") + "\t"
                + "\t" + "\t" + Datas.Roll_error_angleY.ToString("0.00") + "\t"
                + "\t" + "\t" + Datas.Yaw_error_angleZ.ToString("0.00") + "\t"
                + "\t" + "\t" + Datas.Motor1_duty.ToString("0.000")
                + "\t" + "\t" + Datas.Motor2_duty.ToString("0.000")
                + "\t" + "\t" + Datas.Motor3_duty.ToString("0.000")
                + "\t" + "\t" + Datas.Motor4_duty.ToString("0.000")
                + "\t" + "\t" + Datas.RollPitch_Kp_veri.ToString("0.000") + "\t"
                + "\t" + "\t" + Datas.RollPitch_Ki_veri.ToString("0.000") + "\t"
                + "\t" + "\t" + Datas.RollPitch_Kd_veri.ToString("0.000") + "\t"
                + "\t" + "\t" + Datas.Yaw_Kp_veri.ToString("0.000") 
                + "\t" + "\t" + Datas.Yaw_Ki_veri.ToString("0.000") 
                + "\t" + "\t" + Datas.Yaw_Kd_veri.ToString("0.000") 
                + "\t" + "\t" + Datas.Altitude_Kp_veri.ToString("0.000") + "\t"
                + "\t" + "\t" + Datas.Altitude_Ki_veri.ToString("0.000") + "\t"
                + "\t" + "\t" + Datas.Altitude_Kd_veri.ToString("0.000") + "\t"
                + "\t" + "\t" + Datas.Barometer_veri.ToString("0.00") + "\t" + "\t"
                + "\t" + "\t" + Datas.GPS_Lat_veri.ToString() 
                + "\t" + "\t" + Datas.GPS_Lng_veri.ToString() ; 

            StreamWriter sw = new StreamWriter(file, append: true);

            sw.WriteLine(line);

            sw.Close();
        }


    }
}

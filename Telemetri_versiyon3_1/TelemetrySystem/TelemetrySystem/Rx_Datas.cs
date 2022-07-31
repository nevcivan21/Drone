using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TelemetrySystem
{
    class Datas

    {

        public byte[] captured_data = new byte[150];

        static public float Pitch_angleX { get; set; }
        static public float Roll_angleY { get; set; }
        static public float Yaw_angleZ { get; set; }

        static public float Pitch_error_angleX { get; set; }
        static public float Roll_error_angleY { get; set; }
        static public float Yaw_error_angleZ { get; set; }


        static public float Barometer_veri { get; set; }
        static public double GPS_Lat_veri { get; set; }
        static public double GPS_Lng_veri { get; set; }


        static public UInt16 Motor1_duty { get; set; }
        static public UInt16 Motor2_duty { get; set; }
        static public UInt16 Motor3_duty { get; set; }
        static public UInt16 Motor4_duty { get; set; }

        static public float RollPitch_Kp_veri { get; set; }
        static public float RollPitch_Ki_veri { get; set; }
        static public float RollPitch_Kd_veri { get; set; }
        static public float Yaw_Kp_veri { get; set; }
        static public float Yaw_Ki_veri { get; set; }
        static public float Yaw_Kd_veri { get; set; }

        static public float Altitude_Kp_veri { get; set; }
        static public float Altitude_Ki_veri { get; set; }
        static public float Altitude_Kd_veri { get; set; }

        static public float Pitch_integral { get; set; }
        static public float Roll_integral { get; set; }
        static public float Yaw_integral{ get; set; }
        static public float Altitude_İntegral { get; set; }

        static public float Pitch_ControlSignal { get; set; }
        static public float Roll_ControlSignal { get; set; }
        static public float Yaw_ControlSignal { get; set; }
        static public float Altitude_ControlSignal { get; set; }

        static public float Heading { get; set; }

        static public UInt16 KumandaOnOff { get; set; }
        static public UInt16 KumandaFailSafe { get; set; }

        static public UInt16 Error_Flag { get; set; }

        public void DataPackRead(byte[] captured_data, Datas UAV_Datas)
        {
            Datas.Pitch_angleX = BitConverter.ToSingle(captured_data, 2);
            Datas.Roll_angleY = BitConverter.ToSingle(captured_data, 6);
            Datas.Yaw_angleZ = BitConverter.ToSingle(captured_data, 10);

            Datas.RollPitch_Kp_veri = BitConverter.ToSingle(captured_data, 14);
            Datas.RollPitch_Ki_veri = BitConverter.ToSingle(captured_data, 18);
            Datas.RollPitch_Kd_veri = BitConverter.ToSingle(captured_data, 22);
            
            Datas.Yaw_Kp_veri = BitConverter.ToSingle(captured_data, 26);
            Datas.Yaw_Ki_veri = BitConverter.ToSingle(captured_data, 30);
            Datas.Yaw_Kd_veri = BitConverter.ToSingle(captured_data, 34);
            
            Datas.Altitude_Kp_veri = BitConverter.ToSingle(captured_data, 38);
            Datas.Altitude_Ki_veri = BitConverter.ToSingle(captured_data, 42);
            Datas.Altitude_Kd_veri = BitConverter.ToSingle(captured_data, 46);
            
            Datas.Motor1_duty = BitConverter.ToUInt16(captured_data, 50);
            Datas.Motor2_duty = BitConverter.ToUInt16(captured_data, 52);
            Datas.Motor3_duty = BitConverter.ToUInt16(captured_data, 54);
            Datas.Motor4_duty = BitConverter.ToUInt16(captured_data, 56);

            Datas.Barometer_veri = BitConverter.ToSingle(captured_data, 58);
            Datas.GPS_Lat_veri = (float)BitConverter.ToUInt32(captured_data, 62) / 10000000;        // kaç sıfıra bölğünceğini ayarlar.!!
            Datas.GPS_Lng_veri = (float)BitConverter.ToUInt32(captured_data, 66) / 100000000;

            Datas.Pitch_error_angleX = BitConverter.ToSingle(captured_data, 70);
            Datas.Roll_error_angleY = BitConverter.ToSingle(captured_data, 74);
            Datas.Yaw_error_angleZ = BitConverter.ToSingle(captured_data, 78);
            
            /*
            Datas.Pitch_integral = BitConverter.ToSingle(captured_data, 82);
            Datas.Roll_integral = BitConverter.ToSingle(captured_data, 86);
            Datas.Yaw_integral = BitConverter.ToSingle(captured_data, 90);

            Datas.Pitch_ControlSignal = BitConverter.ToSingle(captured_data, 94);
            Datas.Roll_ControlSignal = BitConverter.ToSingle(captured_data, 98);
            Datas.Yaw_ControlSignal = BitConverter.ToSingle(captured_data, 102);

            
            Datas.Heading = BitConverter.ToSingle(captured_data, 106);
            
            Datas.KumandaOnOff = BitConverter.ToUInt16(captured_data, 110);     
            Datas.KumandaFailSafe = BitConverter.ToUInt16(captured_data, 112);

            Datas.Altitude_ControlSignal = BitConverter.ToSingle(captured_data, 114);
            Datas.Altitude_İntegral = BitConverter.ToSingle(captured_data, 118);

            //Datas.Error_Flag = BitConverter.ToUInt16(captured_data, 122);
            */
        }
    }
}

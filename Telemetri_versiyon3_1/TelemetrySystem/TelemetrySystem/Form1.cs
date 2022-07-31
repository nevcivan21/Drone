using GMap.NET.MapProviders;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using GMap.NET;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;
using System.Threading;
using ZedGraph;
using System.Diagnostics;
using System.Drawing.Drawing2D;

namespace TelemetrySystem
{
    public partial class Form1 : Form
    {

        private static PointLatLng start1 = new PointLatLng(41.0257612, 28.88924095); //ortabahçe kordinatlar


        GraphPane myPane = new GraphPane();   //grafik isimi değiştirme tanımlama için.
        LineItem myPitch_Error_Line;                   //çizgilerin tanımlamaaları.
        LineItem myPitch_Line;
        RollingPointPairList listPointsPitch = new RollingPointPairList(60000);
        RollingPointPairList listPointsRoll = new RollingPointPairList(60000);          // değiştir isimleri
        LineItem myPitch_integral_Line;                   //çizgilerin tanımlamaaları.
        LineItem myPitch_Control_Line;
        RollingPointPairList listPointsPitchintegralLine = new RollingPointPairList(60000);
        RollingPointPairList listPointsPitchControl = new RollingPointPairList(60000);

       
        GraphPane myPane2 = new GraphPane();   //grafik isimi değiştirme tanımlama için.
        LineItem myRoll_Error_Line;                   //çizgilerin tanımlamaaları.
        LineItem myRoll_Line;
        RollingPointPairList listPointsPitch2 = new RollingPointPairList(60000);
        RollingPointPairList listPointsRoll2 = new RollingPointPairList(60000);
        LineItem myRoll_integral_Line;                   //çizgilerin tanımlamaaları.
        LineItem myRoll_Control_Line;
        RollingPointPairList listPointsRollintegralLine = new RollingPointPairList(60000);
        RollingPointPairList listPointsRollControl = new RollingPointPairList(60000);

        GraphPane myPane3 = new GraphPane();   //grafik isimi değiştirme tanımlama için.
        LineItem myYaw_Line;                   //çizgilerin tanımlamaaları.
        LineItem myYaw_Error_Line;
        RollingPointPairList listPointsPitch3 = new RollingPointPairList(60000);
        RollingPointPairList listPointsRoll3 = new RollingPointPairList(60000);
        LineItem myYaw_integral_Line;                   //çizgilerin tanımlamaaları.
        LineItem myYaw_Control_Line;
        RollingPointPairList listPointsYawintegralLine = new RollingPointPairList(60000);
        RollingPointPairList listPointsYawControl = new RollingPointPairList(60000);


        GraphPane myPane4 = new GraphPane();   //grafik isimi değiştirme tanımlama için.
        LineItem myBarometer_Line;                   //çizgilerin tanımlamaaları.
        LineItem myBarometer_Extra_Line;
        RollingPointPairList listPointsBarometer4 = new RollingPointPairList(60000);
        RollingPointPairList listPointsBarometer_Extra4 = new RollingPointPairList(60000);
        LineItem Altitude_integral_Line;
        LineItem Altitude_Control_Line;
        RollingPointPairList listPointsAltitudeintegralLine4 = new RollingPointPairList(60000);
        RollingPointPairList listPointsAltitudeControl4 = new RollingPointPairList(60000);


        

        Stopwatch timer = new Stopwatch();

        int end_of_the_packet = 0;
        int MotorDuty1_Buffer=0;
        int MotorDuty3_Buffer = 0;

        //*****************************************Threadler***************************************************

        Thread th1;
        Thread th3_Grafik;
        Thread th4_gerial;
        Thread th5_GPS_Update;
        Thread th6_Flight_Plane;

        //*******************************************Classlar**********************************************************


        Datas datas = new Datas();          
        Logs logs = new Logs(@"C:\\denemee\\");

        //**************************************************************************************************************

        byte[] X_mydizi = new byte[13];
        byte[] Y_mydizi = new byte[36];
        byte[] Z_mydizi = new byte[45];
        double latot, lngot;

        GMapRoute line_layer_Upp;
        GMapOverlay routersOverlay_Upp;

        GMapRoute line_layer_Add;
        GMapOverlay routersOverlay_Add;

        private List<PointLatLng> _points;
        private List<double> _Lat;
        private List<double> _Lng;
        byte[] otolatdizi = new byte[53];
        byte[] Dronekomut_dizi = new byte[58];


        int o = 0;
        int a = 0;
        int LastValue;

        int index = 0;
        int sayac2 = 0;
        int data_counter_buffer = 0;
        int saniyesayac = 0;

        private List<PointLatLng> _inComingLatLng;
        private List<PointLatLng> _inComingLatLng_son;
        private List<double> _inComingLatLngBuffer;
        public Form1()
        {

            InitializeComponent();
            _points = new List<PointLatLng>();
            _Lat = new List<double>();          // gmap eklemeyi unutma!!
            _Lng = new List<double>();
           
            _inComingLatLng = new List<PointLatLng>();
            _inComingLatLngBuffer = new List<double>();
            _inComingLatLng_son =  new List<PointLatLng>();

        }
        
        private void Form1_Load(object sender, EventArgs e)
        {
            CheckForIllegalCrossThreadCalls = false;

            string[] ports = SerialPort.GetPortNames();


            foreach (string port in ports)
            {
                comboBox1.Items.Add(port);
            }
            serialPort1.DataReceived += new SerialDataReceivedEventHandler(SerialPort1_DataReceived);

            Gmap_Start_Position();   //Ortabahçe başlangıç merkezi.
            GrafikHazirla();
            timer1.Start();
            

            gMapControl3.DragButton = MouseButtons.Left;
            gMapControl2.DragButton = MouseButtons.Right;

            gMapControl3.MapProvider = GMapProviders.BingSatelliteMap;
            gMapControl2.MapProvider = GMapProviders.BingSatelliteMap;


        }
        //        ***************************Grafiğin İsim,Boyut Bilgileri*********************************************
        int TickStart1;
        int TickStart2;
        int TickStart3;
        int TickStart4;
        
        private void GrafikHazirla()
        {

            myPane = zedGraphControl1.GraphPane;

            myPane.Title.Text = "Pitch Grafiği";

            myPane.XAxis.Title.Text = "Saniye";
            myPane.YAxis.Title.Text = "Derece";

            myPane.XAxis.Scale.Min = 0;
            myPane.XAxis.Scale.Max = 10;
            myPane.YAxis.Scale.Min = -20;
            myPane.YAxis.Scale.Max = 20;
            TickStart1 = Environment.TickCount;

            

            myPitch_Line = myPane.AddCurve("Pitch_Angle", listPointsPitch, Color.Red, SymbolType.None);
            myPitch_Error_Line = myPane.AddCurve("İstenilen Pitch_Angle", listPointsRoll, Color.Blue, SymbolType.None);
            myPitch_Line.Line.Width = 3;
            myPitch_Error_Line.Line.Width = 3;

            myPitch_integral_Line= myPane.AddCurve("Pitchİntegral", listPointsPitchintegralLine, Color.Green, SymbolType.None);
            myPitch_Control_Line = myPane.AddCurve("Pitch Control Signal", listPointsPitchControl, Color.DarkOrange, SymbolType.None);
            myPitch_integral_Line.Line.Width = 3;
            myPitch_Control_Line.Line.Width = 3;


            myPane2 = zedGraphControl2.GraphPane;
            myPane2.Title.Text = "Roll Grafiği";

            myPane2.XAxis.Title.Text = "Saniye";
            myPane2.YAxis.Title.Text = "Derece";

            myPane2.XAxis.Scale.Min = 0;
            myPane2.XAxis.Scale.Max = 10;
            myPane2.YAxis.Scale.Min = -20;
            myPane2.YAxis.Scale.Max = 20;


            TickStart2 = Environment.TickCount;

            myRoll_Line = myPane2.AddCurve("Roll_Angle", listPointsPitch2, Color.Red, SymbolType.None);
            myRoll_Error_Line = myPane2.AddCurve("İstenilen Roll_Angle", listPointsRoll2, Color.Blue, SymbolType.None);
            myRoll_Line.Line.Width = 3;
            myRoll_Error_Line.Line.Width = 3;
            myRoll_integral_Line = myPane2.AddCurve("Roll integral", listPointsRollintegralLine, Color.Green, SymbolType.None);
            myRoll_Control_Line = myPane2.AddCurve("Roll Control Signal", listPointsRollControl, Color.DarkOrange, SymbolType.None);
            myRoll_integral_Line.Line.Width = 3;
            myRoll_Control_Line.Line.Width = 3;



            myPane3 = zedGraphControl3.GraphPane;
            myPane3.Title.Text = "Yaw Grafiği";

            myPane3.XAxis.Title.Text = "Saniye";
            myPane3.YAxis.Title.Text = "Derece";

            myPane3.XAxis.Scale.Min = 0;
            myPane3.XAxis.Scale.Max = 10;
            myPane3.YAxis.Scale.Min = -90;
            myPane3.YAxis.Scale.Max = 90;


            TickStart3 = Environment.TickCount;

            myYaw_Line = myPane3.AddCurve("Yaw_Angle", listPointsPitch3, Color.Red, SymbolType.None);
            myYaw_Error_Line = myPane3.AddCurve("İstenilen Yaw_Angle", listPointsRoll3, Color.Blue, SymbolType.None);
            myYaw_Line.Line.Width = 3;
            myYaw_Error_Line.Line.Width = 3;
            myYaw_integral_Line = myPane3.AddCurve("Roll integral", listPointsYawintegralLine, Color.Green, SymbolType.None);
            myYaw_Control_Line = myPane3.AddCurve("Roll Control Signal", listPointsYawControl, Color.DarkOrange, SymbolType.None);
            myYaw_integral_Line.Line.Width = 3;
            myYaw_Control_Line.Line.Width = 3;

            myPane4 = zedGraphControl4.GraphPane;
            myPane4.Title.Text = "Kontrol";

            myPane4.XAxis.Title.Text = "Saniye";
            myPane4.YAxis.Title.Text = "Derece";

            myPane4.XAxis.Scale.Min = 0;
            myPane4.XAxis.Scale.Max = 10;
            myPane4.YAxis.Scale.Min = -90;
            myPane4.YAxis.Scale.Max = 90;

            TickStart4 = Environment.TickCount;


            myBarometer_Line = myPane4.AddCurve("Barometer", listPointsBarometer4, Color.Red, SymbolType.None);
            myBarometer_Extra_Line = myPane4.AddCurve("Barometer_Extra", listPointsBarometer_Extra4, Color.Blue, SymbolType.None);
            myBarometer_Line.Line.Width = 3;
            myBarometer_Extra_Line.Line.Width = 3;
            Altitude_integral_Line = myPane4.AddCurve("Altitude_İntegral", listPointsAltitudeintegralLine4, Color.Green, SymbolType.None);
            Altitude_Control_Line = myPane4.AddCurve("Altitude_Control Signal", listPointsAltitudeControl4, Color.Black, SymbolType.None);
            Altitude_integral_Line.Line.Width = 3;
            Altitude_Control_Line.Line.Width = 3;



        }



        //       ***************************Okunan verilerin paketinin çözümlendiği kısım*****************************
        int deneme;
        static int step = 0;
        static int data_counter = 0;
        private void SerialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            if (serialPort1.IsOpen == true)
            {
                try
                {
                    int bytes = serialPort1.BytesToRead;
                    byte[] buffer = new byte[bytes];

                    serialPort1.Read(buffer, 0, bytes);
                    for (int i = 0; i < bytes; i++)
                    {
                        switch (step)
                        {
                            case 0:
                                {
                                    if (buffer[i] == 65)            //Header 1 = 'A'
                                    {

                                        datas.captured_data[data_counter] = buffer[i];
                                        data_counter++;
                                        step = 1;
                                    }
                                    else
                                    {
                                        step = 0;
                                        
                                    }
                                    break;
                                }
                            case 1:
                                {
                                    if (buffer[i] == 66)         //Header 2= 'B'
                                    {

                                        datas.captured_data[data_counter] = buffer[i];
                                        data_counter++;
                                        step = 2;    
                                    }
                                    else
                                    {
                                        step = 1;
                                       
                                    }
                                    break;
                                }
                            case 2:
                                {
                                    datas.captured_data[data_counter] = buffer[i];

                                    data_counter++;
                                    if (data_counter == 82)        
                                    {
                                        //MotorDuty1_Buffer= BitConverter.ToUInt16(buffer, 50);
                                        //MotorDuty3_Buffer = BitConverter.ToUInt16(buffer, 54);

                                       // data_counter_buffer++;
                                       // datas.DataPackRead(buffer, datas);
                                       // data_counter = 0;
                                        //step = 0;

                                        datas.DataPackRead(buffer, datas);

                                        data_counter = 0;
                                        step = 0;
                                        //end_of_the_packet = ((buffer[0] * buffer[1]) + (buffer[110] * buffer[112]) + (21 + 07 + 97) + (MotorDuty1_Buffer + MotorDuty3_Buffer)) / 100;


                                        // if (buffer[124] == end_of_the_packet)
                                        // {
                                        //     data_counter_buffer++;
                                        //    datas.DataPackRead(buffer, datas);
                                        //     data_counter = 0;
                                        //    step = 0;
                                        // }



                                    }

                                    break;
                                }
                           
                            default:
                                break;

                        }

                    }

                }
                catch (Exception )
                {
                    ; ;
                }
            }
        }

       

        public delegate void DelegateStandardPattern();
        private void delegate_func()
        {
            try
            {
                while (true)
                {
                    this.Invoke((MethodInvoker)delegate { displayData_event(); });

                    Thread.Sleep(50);
                }
            }
            catch (Exception)
            {
                ; ;
            }

        }
        private bool Error_btn_baglant = false;

        private void displayData_event()
        {
            if (serialPort1.IsOpen == true)
            {
                try
                {
                   
                        pitch_angle_text.Text = Datas.Pitch_angleX.ToString("0.000");
                        roll_angle_text.Text = Datas.Roll_angleY.ToString("0.000");
                        yaw_angle_text.Text = Datas.Yaw_angleZ.ToString("0.000");

                        PitchError_text.Text = Datas.Pitch_error_angleX.ToString("0.000");
                        RollError_text.Text = Datas.Roll_error_angleY.ToString("0.000");
                        YawError_text.Text = Datas.Yaw_error_angleZ.ToString("0.000");

                        artifical_Horizon1.PitchAngle = Convert.ToDouble(Datas.Pitch_angleX)  ;  //5 
                        artifical_Horizon1.RollAngle = Convert.ToDouble(Datas.Roll_angleY)  ;    //12
                        artifical_Horizon1.Update();

                        RollPitch_kp_veri_text.Text = Datas.RollPitch_Kp_veri.ToString("0.000");
                        RollPitch_ki_veri_Text.Text = Datas.RollPitch_Ki_veri.ToString("0.000");
                        RollPitch_kd_veri_text.Text = Datas.RollPitch_Kd_veri.ToString("0.000");

                        

                        Yaw_kp_veri_text.Text = Datas.Yaw_Kp_veri.ToString("0.000");
                        Yaw_ki_veri_Text.Text = Datas.Yaw_Ki_veri.ToString("0.000");
                        Yaw_kd_veri_text.Text = Datas.Yaw_Kd_veri.ToString("0.000");

                        Altitude_kp_veri_text.Text = Datas.Altitude_Kp_veri.ToString("0.000");
                        Altitude_ki_veri_Text.Text = Datas.Altitude_Ki_veri.ToString("0.000");
                        Altitude_kd_veri_text.Text = Datas.Altitude_Kd_veri.ToString("0.000");

                        motor1_duty_text.Text = Datas.Motor1_duty.ToString();
                        motor2_duty_text.Text = Datas.Motor2_duty.ToString();
                        motor3_duty_text.Text = Datas.Motor3_duty.ToString();
                        motor4_duty_text.Text = Datas.Motor4_duty.ToString();

                        textBox4_m1.Text = Datas.Motor1_duty.ToString();
                        textBox5_m2.Text = Datas.Motor2_duty.ToString();
                        textBox6_m3.Text = Datas.Motor3_duty.ToString();
                        textBox3_m4.Text = Datas.Motor4_duty.ToString();


                        Barometer_Text.Text = Datas.Barometer_veri.ToString("0.000");
                        txtLat.Text = Datas.GPS_Lat_veri.ToString();
                        txtLong.Text = Datas.GPS_Lng_veri.ToString();

                        txt_PitchControl.Text = Datas.Pitch_ControlSignal.ToString();
                        txt_Pitchintegral.Text = Datas.Pitch_integral.ToString();

                        txt_RollSignal.Text = Datas.Roll_ControlSignal.ToString();
                        txt_Roll_İntegral.Text = Datas.Roll_integral.ToString();

                        txt_YawSignal.Text = Datas.Yaw_ControlSignal.ToString();
                        txt_Yawintegral.Text = Datas.Yaw_integral.ToString();
                            
                    
                        txt_AltitudeControl.Text = Datas.Altitude_ControlSignal.ToString();
                        txt_Altitudeİntegral.Text = Datas.Altitude_İntegral.ToString();
                    
                    /*
                        if (Datas.KumandaOnOff == 1)
                        {
                            txt_OnnOff.BackColor = Color.DarkGreen;
                            txt_OnnOff.Text = "Kumanda On";


                        }
                        if (Datas.KumandaOnOff == 0)
                        {
                            txt_OnnOff.BackColor = Color.DarkRed;
                            txt_OnnOff.Text = "Kumanda Off";

                        }
                        if (Datas.KumandaFailSafe == 1)
                        {
                             txt_Failsafe.BackColor = Color.DarkGreen;
                             txt_Failsafe.Text = "FailSafe On";

                        }
                        if (Datas.KumandaFailSafe == 0)
                        {
                            txt_Failsafe.BackColor = Color.DarkRed;
                            txt_Failsafe.Text = "FailSafe Off";

                        }

                     
                    if (Datas.Error_Flag == 1)
                    {
                        txt_errorflag.Text = "Bağlantı Koptu";
                        txt_errorflag.BackColor = Color.DarkRed;
                        txt_errorflag.Visible = true;
                        picturebx_error.Visible = true;
                       
                        MessageBox.Show("Bağlantı koptu,Sistemi Resetleyip Yeniden Başlatınız.", "Bağlantı Hatası", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    }
                    
                    if (Datas.Error_Flag == 0)
                    {

                        txt_errorflag.Visible = false;
                        picturebx_error.Visible = false;
                    }
                     */
                    
                    

                        GPS_Update_new();
                        Draw_Grapichs();
                        


                        TimeSpan timeTaken = timer.Elapsed;

                        string timeStr = timeTaken.ToString();
                        string parts = timeStr.Split('.')[0];       //Burada bize saat,dakika ,saniye ve milisaniye cinsinden zamanı veren bir fonksiyon var.
                                                                    //Buradaki split fonksiyonu "." ile ayrılmış gelen verileri ayrıştırır.Biz burda (0) yani saat dakika ve saniye bilgilerini aldık sadece.
                         if (sayac2 > 15)
                         {
                            logs.WriteLine(parts, saattxt.Text);
                         }
                        
                        GPS_Flight_Plane();

                        txtucus.Text = Convert.ToString(parts);
                        saattxt.Text = DateTime.Now.ToLongTimeString();
                }
                catch (Exception dedede)
                {
                    
                    MessageBox.Show(dedede.ToString());
                    ; ;
                   
                }
            }

            
            if(serialPort1.IsOpen == false)
            {
                txt_errorflag.Text = "Bağlantı Koptu";
                txt_errorflag.BackColor = Color.DarkRed;
                txt_errorflag.Visible = true;
                picturebx_error.Visible = true;
                MessageBox.Show("Bağlantı koptu,Bağlantıları kontrol edin.", "Bağlantı Hatası", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                Error_btn_baglant = true;

                if (Error_btn_baglant == true)
                {
                   // Oto_Error_Function();
                    txt_errorflag.Visible = false;
                    picturebx_error.Visible = false;
                    Error_btn_baglant = false;

                }

            }
            
           
        }
       

        //****************************Formu Kapatırken DataReceivedi Kapatmak için kullanılan kısım****************************
        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            if (serialPort1.IsOpen) serialPort1.Close();

            serialPort1.DataReceived -= new SerialDataReceivedEventHandler(SerialPort1_DataReceived);

        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (serialPort1.IsOpen) serialPort1.Close();

            serialPort1.DataReceived -= new SerialDataReceivedEventHandler(SerialPort1_DataReceived);
        }



        //    *******************************************Gelen değerlerin Grafikte Gösterilmesi***************************

        
        private void Draw_Grapichs()
        {
            try
            {
                if (_scrolling)
                {

                    double time1 = (Environment.TickCount - TickStart1) / 1000.0;               
                    Scale xScale1 = zedGraphControl1.GraphPane.XAxis.Scale;
                    
                    if (time1 > xScale1.Max - xScale1.MajorStep)
                    {
                        xScale1.Max = time1 + xScale1.MajorStep;
                        xScale1.Min = xScale1.Max - 10;//Auto scale x axis in limit time
                    }
                    zedGraphControl1.AxisChange();
                    

                    if (checkbox_Pitchint.Checked == false && checkBox_PitchCntrl.Checked == false )
                    {
                        
                        listPointsPitch.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_angleX.ToString())));  
                        listPointsRoll.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_error_angleX.ToString())));    
                        myPane.XAxis.Scale.Max = time1;
                        myPane.AxisChange();
                        zedGraphControl1.Refresh();
                        
                    }


                    if (checkbox_Pitchint.Checked == true && checkBox_PitchCntrl.Checked == false)
                    {

                        listPointsPitch.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_angleX.ToString())));  
                        listPointsRoll.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_error_angleX.ToString())));     
                        listPointsPitchintegralLine.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_integral.ToString())));
                        
                        myPane.XAxis.Scale.Max = time1;
                        myPane.AxisChange();
                        zedGraphControl1.Refresh();
                        
                    }

                    if (checkbox_Pitchint.Checked == true && checkBox_PitchCntrl.Checked == true)
                    {
                      
                        listPointsPitch.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_angleX.ToString()))); 
                        listPointsRoll.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_error_angleX.ToString())));    
                        listPointsPitchintegralLine.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_integral.ToString())));
                        listPointsPitchControl.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_ControlSignal.ToString())));
                        myPane.XAxis.Scale.Max = time1;
                        myPane.AxisChange();
                        zedGraphControl1.Refresh();
                    }

                    if (checkbox_Pitchint.Checked == false && checkBox_PitchCntrl.Checked == true)
                    {

                        listPointsPitch.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_angleX.ToString())));  
                        listPointsRoll.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_error_angleX.ToString())));     
                        listPointsPitchControl.Add(new PointPair(time1, Convert.ToDouble(Datas.Pitch_ControlSignal.ToString())));
                        myPane.XAxis.Scale.Max = time1;
                        myPane.AxisChange();
                        zedGraphControl1.Refresh();
                    }


                    double time2 = (Environment.TickCount - TickStart2) / 1000.0;              
                    Scale xScale2 = zedGraphControl2.GraphPane.XAxis.Scale;
                    if (time2 > xScale2.Max - xScale2.MajorStep)
                    {
                        xScale2.Max = time2 + xScale2.MajorStep;
                        xScale2.Min = xScale2.Max - 10;//Auto scale x axis in limit time
                    }
                    zedGraphControl2.AxisChange();

                    if (checkBox_Rollintegral.Checked==false && checkBox_RollCntrl.Checked == false)
                    {
                        listPointsPitch2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_angleY.ToString())));  // pitch angeli yazdıyor
                        listPointsRoll2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_error_angleY.ToString())));     //roll angeli yazdırıyor
                        myPane2.XAxis.Scale.Max = time2;
                        myPane2.AxisChange();
                        zedGraphControl2.Refresh();
                    }
                    if (checkBox_Rollintegral.Checked == true && checkBox_RollCntrl.Checked == false)
                    {
                        listPointsPitch2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_angleY.ToString())));  
                        listPointsRoll2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_error_angleY.ToString())));     
                        listPointsRollintegralLine.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_integral.ToString())));
                        myPane2.XAxis.Scale.Max = time2;
                        myPane2.AxisChange();
                        zedGraphControl2.Refresh();
                    }
                    if (checkBox_Rollintegral.Checked == false && checkBox_RollCntrl.Checked == true)
                    {
                        listPointsPitch2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_angleY.ToString())));  // pitch angeli yazdıyor
                        listPointsRoll2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_error_angleY.ToString())));     //roll angeli yazdırıyor
                        listPointsRollControl.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_ControlSignal.ToString())));
                        myPane2.XAxis.Scale.Max = time2;
                        myPane2.AxisChange();
                        zedGraphControl2.Refresh();
                    }
                    if (checkBox_Rollintegral.Checked == true && checkBox_RollCntrl.Checked == true)
                    {
                        listPointsPitch2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_angleY.ToString())));  
                        listPointsRoll2.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_error_angleY.ToString())));     
                        listPointsRollintegralLine.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_integral.ToString())));
                        listPointsRollControl.Add(new PointPair(time2, Convert.ToDouble(Datas.Roll_ControlSignal.ToString())));
                        myPane2.XAxis.Scale.Max = time2;
                        myPane2.AxisChange();
                        zedGraphControl2.Refresh();
                    }

                    double time3 = (Environment.TickCount - TickStart3) / 1000.0;              
                    Scale xScale3 = zedGraphControl3.GraphPane.XAxis.Scale;
                    if (time3 > xScale3.Max - xScale3.MajorStep)
                    {
                        xScale3.Max = time3 + xScale3.MajorStep;
                        xScale3.Min = xScale3.Max - 10;//Auto scale x axis in limit time
                    }
                    zedGraphControl3.AxisChange();

                    if (checkBox_Yawintegral.Checked == false && checkBox_YawCntrl.Checked == false)
                    {
                        listPointsPitch3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_angleZ.ToString())));  
                        listPointsRoll3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_error_angleZ.ToString())));     
                        myPane3.XAxis.Scale.Max = time3;
                        myPane3.AxisChange();
                        zedGraphControl3.Refresh();
                        
                    }

                    if (checkBox_Yawintegral.Checked == true && checkBox_YawCntrl.Checked == false)
                    {
                        listPointsPitch3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_angleZ.ToString())));  
                        listPointsRoll3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_error_angleZ.ToString())));    
                        listPointsYawintegralLine.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_integral.ToString())));
                        myPane3.XAxis.Scale.Max = time3;
                        myPane3.AxisChange();
                        zedGraphControl3.Refresh();
                    }

                    if (checkBox_Yawintegral.Checked == false && checkBox_YawCntrl.Checked == true)
                    {
                        listPointsPitch3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_angleZ.ToString())));  
                        listPointsRoll3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_error_angleZ.ToString())));     
                        listPointsYawControl.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_ControlSignal.ToString())));
                        myPane3.XAxis.Scale.Max = time3;
                        myPane3.AxisChange();
                        zedGraphControl3.Refresh();
                    }
                    if (checkBox_Yawintegral.Checked == true && checkBox_YawCntrl.Checked == true)
                    {
                        listPointsPitch3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_angleZ.ToString())));  
                        listPointsRoll3.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_error_angleZ.ToString())));     
                        listPointsYawintegralLine.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_integral.ToString())));
                        listPointsYawControl.Add(new PointPair(time3, Convert.ToDouble(Datas.Yaw_ControlSignal.ToString())));
                        myPane3.XAxis.Scale.Max = time3;
                        myPane3.AxisChange();
                        zedGraphControl3.Refresh();
                    }

                    
                    double time4 = (Environment.TickCount - TickStart4) / 1000.0;              
                    Scale xScale4 = zedGraphControl4.GraphPane.XAxis.Scale;
                    if (time4 > xScale4.Max - xScale4.MajorStep)
                    {
                        xScale4.Max = time4 + xScale4.MajorStep;
                        xScale4.Min = xScale4.Max - 10;//Auto scale x axis in limit time
                    }
                    
                    zedGraphControl4.AxisChange();

                    if (checkBox_altitudeintegral.Checked == true && checkBox_altitudesignal.Checked == false)
                    {
                        listPointsBarometer4.Add(new PointPair(time4, Convert.ToDouble(Datas.Barometer_veri.ToString())));
                        listPointsAltitudeintegralLine4.Add(new PointPair(time4, Convert.ToDouble(Datas.Altitude_İntegral.ToString())));
                        myPane4.XAxis.Scale.Max = time4;
                        myPane4.AxisChange();
                        zedGraphControl4.Refresh();
                    }

                    if (checkBox_altitudeintegral.Checked == false && checkBox_altitudesignal.Checked == true)
                    {
                        listPointsBarometer4.Add(new PointPair(time4, Convert.ToDouble(Datas.Barometer_veri.ToString())));
                        listPointsAltitudeControl4.Add(new PointPair(time4, Convert.ToDouble(Datas.Altitude_ControlSignal.ToString())));
                        myPane4.XAxis.Scale.Max = time4;
                        myPane4.AxisChange();
                        zedGraphControl4.Refresh();
                    }
                    if (checkBox_altitudeintegral.Checked == true && checkBox_altitudesignal.Checked == true)
                    {
                        listPointsBarometer4.Add(new PointPair(time4, Convert.ToDouble(Datas.Barometer_veri.ToString())));
                        listPointsAltitudeintegralLine4.Add(new PointPair(time4, Convert.ToDouble(Datas.Altitude_İntegral.ToString())));
                        listPointsAltitudeControl4.Add(new PointPair(time4, Convert.ToDouble(Datas.Altitude_ControlSignal.ToString())));
                        myPane4.XAxis.Scale.Max = time4;
                        myPane4.AxisChange();
                        zedGraphControl4.Refresh();
                    }
                    if (checkBox_altitudeintegral.Checked == false && checkBox_altitudesignal.Checked == false)
                    {
                        listPointsBarometer4.Add(new PointPair(time4, Convert.ToDouble(Datas.Barometer_veri.ToString())));  
                        
                        myPane4.XAxis.Scale.Max = time4;
                        myPane4.AxisChange();
                        zedGraphControl4.Refresh();
                    }

                   

                    
                }
               

                
            }
            catch (Exception)
            {

                ; ;
            }
        }
        
        
       

        // ***************************************Bağlan Butonunun Eventi.*****************************************

        private void btnconnect_Click(object sender, EventArgs e)    //Seriporta bağlanmak için kullandığımız butondur.
        {
            timer.Start();

            try
            {
                if(serialPort1.IsOpen == true)                      //eğer seri port açık ise ;
                {
                    return;
                }
                th1 = new Thread(delegate_func);
                //th1.Priority = ThreadPriority.BelowNormal;
                th1.IsBackground = true;
                th1.Start();

                
                th3_Grafik = new Thread(Draw_Grapichs);
                th3_Grafik.Start();

                th5_GPS_Update = new Thread(GPS_Update_new);
                th5_GPS_Update.Start();

                th6_Flight_Plane = new Thread(GPS_Flight_Plane);
                th6_Flight_Plane.Start();

                serialPort1.PortName = comboBox1.Text;
                
                serialPort1.BaudRate = int.Parse("9600");
                serialPort1.DataBits = int.Parse("8");
                serialPort1.StopBits = StopBits.One;
                serialPort1.Parity = Parity.None;
                //serialPort1.ReadBufferSize = 80;                    //Buffer boyutunu belirleriz.
                
                btnconnect.Enabled = false;
                richTextBox1.Text = "Bağlantı Başarılı";
                richTextBox1.ForeColor = Color.Green;
                serialPort1.ReadTimeout = 200;                      //Başlangıçta sistemi açarken sisteme küçük bir zaman tanıyoruz.
                serialPort1.Open();
                DiscardInBuffer();
                DiscardOutBuffer();
               

            }
            catch (Exception ex)
            {

                MessageBox.Show(ex.Message, "haaata");

            }
        }
        /*
        private void Oto_Error_Function()
        {
            try
            {
                serialPort1.PortName = comboBox1.Text;
                serialPort1.BaudRate = int.Parse("9600");
                serialPort1.DataBits = int.Parse("8");
                serialPort1.StopBits = StopBits.One;
                serialPort1.Parity = Parity.None;

                serialPort1.ReadTimeout = 200;                      

                serialPort1.Open();
                DiscardInBuffer();
                DiscardOutBuffer();
            }
            catch (Exception er)
            {
                
                MessageBox.Show(er.Message, "Port Hatası");
            }
            
        
        }
        */

        // ********************************Kapat Buton Eventi.*****************************************
        private void button3_Click(object sender, EventArgs e)              //Bağlantıyı kesen buton.
        {
            th1.Abort();
            
            th3_Grafik.Abort();
            
            if (th4_gerial != null)     
            {                               
                th4_gerial.Abort();
            }
            
            th5_GPS_Update.Abort();
            th6_Flight_Plane.Abort();

            serialPort1.Close();
            btnconnect.Enabled = true;

            richTextBox1.Text = "Bağlantı Kapatıldı";
            richTextBox1.ForeColor = Color.Red;
            
        }
        //**********************************Yenile Butonu Eventi*************************************************

        private void yenile_button_Click_1(object sender, EventArgs e)                  //Serİportları yenilemek için kullandığımız buton.
        {
            comboBox1.Items.Clear();
            comboBox1.ResetText();
            baudrate.ResetText();
            string[] ports = SerialPort.GetPortNames();
            foreach (string port in ports)
            {
                comboBox1.Items.Add(port);
            }
            richTextBox1.Text = "Portlar yenilendi.";
            richTextBox1.ForeColor = Color.Blue;
            btnconnect.Enabled = true;
        }

       
        //******************************** PID Değerlerini Göndermek için Kullanılan Kısmın Ayarları*************************
        private void RollPitch_Kp_text_KeyPress(object sender, KeyPressEventArgs e)         //PID değerlerini girerken sadece sayı ve "," girmesi için koruma.
        {
            if (!char.IsControl(e.KeyChar)
                    && !char.IsDigit(e.KeyChar)
                     && e.KeyChar != ',')
            {
                e.Handled = true;

            }
        }

        private void Yaw_Kp_text_KeyPress(object sender, KeyPressEventArgs e)               //PID değerlerini girerken sadece sayı ve "," girmesi için koruma.
        {
            if (!char.IsControl(e.KeyChar)
                    && !char.IsDigit(e.KeyChar)
                     && e.KeyChar != ',')
            {
                e.Handled = true;

            }
        }

        private void Altitude_Kp_text_KeyPress(object sender, KeyPressEventArgs e)          //PID değerlerini girerken sadece sayı ve "," girmesi için koruma.
        {
            if (!char.IsControl(e.KeyChar)
                    && !char.IsDigit(e.KeyChar)
                     && e.KeyChar != ',')
            {
                e.Handled = true;

            }
        }

        //******************************************PID Değer Gönderme Buton Eventleri**************************************
        
        private void X_set_params_button_Click_1(object sender, EventArgs e)            //RollPitch PID değerleri Gönderme Fonk.
        {
            
            float X_Kp, X_Ki, X_Kd;
            X_Kp = float.Parse(RollPitch_Kp_text.Text);    //verileri virgüllü göndermek için parseliyoruz.
            
            X_Ki = float.Parse(RollPitch_Ki_text.Text);
            X_Kd = float.Parse(RollPitch_Kd_text.Text);
            
            if (serialPort1.IsOpen)
            {
                    var X_Kp_bytes = new byte[4];
                    var X_Ki_bytes = new byte[4];
                    var X_Kd_bytes = new byte[4];
                    X_Kp_bytes = BitConverter.GetBytes(X_Kp);
                    X_Ki_bytes = BitConverter.GetBytes(X_Ki);

                    X_Kd_bytes = BitConverter.GetBytes(X_Kd);

                    X_Kp_bytes.CopyTo(X_mydizi, 1);    // Verileri mydizinin içine atıyoruz seriporttan göndermek için.
                    X_Ki_bytes.CopyTo(X_mydizi, 5);
                    X_Kd_bytes.CopyTo(X_mydizi, 9);

                    X_mydizi[0] = (byte)'R';

                    serialPort1.Write(X_mydizi, 0, X_mydizi.Length);

            }
            
        }
        private void Y_set_params_button_Click(object sender, EventArgs e)          //Yaw PID değerlerini göndermek için kullanılan buton fonksiyonu.
        {

            float Y_Kp, Y_Ki, Y_Kd;                                                 //Virgüllü verileri tutmak için tanımlanan değişkenler.
            Y_Kp = float.Parse(Yaw_Kp_text.Text);                                   //verileri virgüllü göndermek için parseliyoruz.
            Y_Ki = float.Parse(Yaw_Ki_text.Text);
            Y_Kd = float.Parse(Yaw_Kd_text.Text);

            if (serialPort1.IsOpen)
            {
                    var Y_Kp_bytes = new byte[4];                                   //Verileri 4 byte'ta aldığımız için,verileri tutmak için 4 bytelık 3 adet dizi tanımlandı.3 değer olduğu için.
                    var Y_Ki_bytes = new byte[4];
                    var Y_Kd_bytes = new byte[4];
                    Y_Kp_bytes = BitConverter.GetBytes(Y_Kp);                       //Verileri Bitconverter ile 4 bytlık dizinin içine dönüştürüp atıyoruz.
                    Y_Ki_bytes = BitConverter.GetBytes(Y_Ki);
                    Y_Kd_bytes = BitConverter.GetBytes(Y_Kd);

                    Y_Kp_bytes.CopyTo(Y_mydizi, 13);                                // Verileri Y_mydizinin içine atıyoruz seriporttan göndermek için.
                    Y_Ki_bytes.CopyTo(Y_mydizi, 17);                                //Verileri 4 byte'lık uzunluklarla 13'ten başlayarak yükledil.
                    Y_Kd_bytes.CopyTo(Y_mydizi, 21);                                //Çünkü RollPitch değerleri 0 ile 12 arasında Altitude ise 24 ile 26 arasında.
                    Y_mydizi[0] = (byte)'Y';                                        //İşlemcide veriyi alırken YAW değerinin olduğunu anlamak için "0" byte'ına 'Y' headırını atadık.

                    serialPort1.Write(Y_mydizi, 0, Y_mydizi.Length);                //Verileri Gönderiyoruz.

            }
        }
        private void Z_set_params_button_Click(object sender, EventArgs e)              //Altitude PID değerlerini göndermek için kullanılan buton fonksiyonu.
        {

            float Z_Kp, Z_Ki, Z_Kd;
            Z_Kp = float.Parse(Altitude_Kp_text.Text);    //verileri virgüllü göndermek için parseliyoruz.
            Z_Ki = float.Parse(Altitude_Ki_text.Text);
            Z_Kd = float.Parse(Altitude_Kd_text.Text);

            if (serialPort1.IsOpen)
            {
                    var Z_Kp_bytes = new byte[4];
                    var Z_Ki_bytes = new byte[4];
                    var Z_Kd_bytes = new byte[4];
                    Z_Kp_bytes = BitConverter.GetBytes(Z_Kp);
                    Z_Ki_bytes = BitConverter.GetBytes(Z_Ki);
                    Z_Kd_bytes = BitConverter.GetBytes(Z_Kd);

                    Z_Kp_bytes.CopyTo(Z_mydizi, 25);    // Verileri mydizinin içine atıyoruz seriporttan göndermek için.
                    Z_Ki_bytes.CopyTo(Z_mydizi, 29);
                    Z_Kd_bytes.CopyTo(Z_mydizi, 33);
                    Z_mydizi[0] = (byte)'L';

                    serialPort1.Write(Z_mydizi, 0, Z_mydizi.Length);

            }
        }

        //*******************************************Haritanın İlk Konumunun Belirlenmesi***************************************

        private void Gmap_Start_Position()                                      //Arayüzü açtığımızda haritanın start pozisyonunu belirleyen fonksiyon.
        {
            gMapControl3.ShowCenter = false;
            //Bitmap bmpMarker = (Bitmap)Image.FromFile("img/icon.ICO");

            GMapMarker marker = new GMarkerGoogle(start1, GMarkerGoogleType.red_dot);
            GMapOverlay markers = new GMapOverlay("markers");
            markers.Markers.Add(marker);
            gMapControl3.Position = new GMap.NET.PointLatLng(41.0257612, 28.88924095);      //Ortabahçe kordinatları.
            gMapControl3.Overlays.Add(markers);
            gMapControl3.DragButton = MouseButtons.Middle;
            gMapControl3.MapProvider = GMapProviders.BingSatelliteMap;
            gMapControl3.MinZoom = 10;
            gMapControl3.MaxZoom = 150;
            gMapControl3.Zoom = 18;

            gMapControl2.Position = new GMap.NET.PointLatLng(41.0257612, 28.88924095);
            gMapControl2.MinZoom = 10;
            gMapControl2.MaxZoom = 150;
            gMapControl2.Zoom = 18;


        }
        

        //********************************************ConvertToDegrees*******************************************
        public static double ConvertRadiansToDegrees(double radians)
        {
            double degrees = (180 / Math.PI) * radians;
            return (degrees);
        }


        //*************************************Marker Ekleme Fonksiyonu****************************************

        GMapOverlay markers_add = new GMapOverlay("markers");       //SAĞDAKİ HARİTA İÇİN.
        GMapOverlay markers_upp = new GMapOverlay("markers");       //Soldaki takip haritası için.
        private void AddMarker(PointLatLng mouse_position)      
        {
            PointLatLng receivedPosition = new PointLatLng(mouse_position.Lat, mouse_position.Lng);   // burdaki mouse position yrine gelen verileri yaz gps takip etsin.
            //Bitmap bmpMarker = (Bitmap)Image.FromFile("img/yellow.ICO");   //sarı ikon
            GMapMarker marker = new GMarkerGoogle(receivedPosition, GMarkerGoogleType.red_small);
            //GMapOverlay markers = new GMapOverlay("markers");

            markers_add.Markers.Add(marker);
            gMapControl2.Overlays.Add(markers_add);


            gMapControl2.Zoom += 0.00000001;
            gMapControl2.Zoom -= 0.00000001;
        }

       
        //********************************************GPS Veri Takibi******************************************
        private void GPS_Update_new()           //GPS verilerini haritada takip etmek için kullanıyon fonksiyon.
        {
            

            _inComingLatLng.Add(new PointLatLng(Datas.GPS_Lat_veri, Datas.GPS_Lng_veri)); //gelenleri alıyor     (dene sıkıntı yoksa sil)
            _inComingLatLngBuffer.Add(Convert.ToDouble(Datas.GPS_Lat_veri));    // geleni inte atıyo

            if (_inComingLatLngBuffer[index] != 0)  // gelen sıfırdan farklı mı diye bakıp asıl olana atıyor
            {
                _inComingLatLng_son.Add(new PointLatLng(Datas.GPS_Lat_veri, Datas.GPS_Lng_veri));
                
            }
            
           

            Bitmap bmpMarker = (Bitmap)Image.FromFile("img/ne.PNG");   //sarı ikon

            PointLatLng receivedPosition = new PointLatLng(Datas.GPS_Lat_veri, Datas.GPS_Lng_veri);   // burdaki mouse position yrine gelen verileri yaz gps takip etsin.
            gMapControl3.Position = new PointLatLng(Datas.GPS_Lat_veri, Datas.GPS_Lng_veri);
            GMapMarker marker = new GMarkerGoogle(receivedPosition, bmpMarker);


            gMapControl3.Zoom += 0.00000001;
            gMapControl3.Zoom -= 0.00000001;

            
            if (routersOverlay_Upp != null)                 
            {
                markers_upp.Markers.Clear();
                routersOverlay_Upp.Routes.Clear();
                gMapControl3.Overlays.Clear();
            }

            markers_upp.Markers.Add(marker);
            gMapControl3.Overlays.Add(markers_upp);

            
            line_layer_Upp = new GMapRoute(_inComingLatLng_son, "single_line");
            line_layer_Upp.Stroke = new Pen(Brushes.Red, 4); //width and color of line
            routersOverlay_Upp = new GMapOverlay("routes");
            routersOverlay_Upp.Routes.Add(line_layer_Upp);
            gMapControl3.Overlays.Add(routersOverlay_Upp);

            gMapControl3.UpdateRouteLocalPosition(line_layer_Upp);
            
            index++;
            
        }
        
        //********************************************Harita Üzerindeki mouseClick Eventinin Olduğu kısım.*****************************
       
        private void gMapControl2_MouseClick(object sender, MouseEventArgs e)
        {
            //Mouse'nin tıkladığı yerdeki kordinatları belirleyip harita üzerinde işaretleme yapar.
            if (e.Button == MouseButtons.Left)
            {
                PointLatLng mouse_position = gMapControl2.FromLocalToLatLng(e.X, e.Y);
                
                var point = gMapControl2.FromLocalToLatLng(e.X, e.Y);
                AddMarker(mouse_position);
                Double lat = point.Lat;
                double lng = point.Lng;

                

                _points.Add(new PointLatLng(lat, lng));
                _Lat.Add(Convert.ToDouble(lat));        //Gelen lat değerleri bu dizi içerisinde tutuluyor. 
                _Lng.Add(Convert.ToDouble(lng));        //Gelen lng değerleri bu dizi içerisinde tutuluyor.


                LastValue = (_Lng.Count);


                line_layer_Add = new GMapRoute(_points, "single_line");
                line_layer_Add.Stroke = new Pen(Brushes.Blue, 4); //width and color of line
                routersOverlay_Add = new GMapOverlay("routes");
                routersOverlay_Add.Routes.Add(line_layer_Add);
                gMapControl2.Overlays.Add(routersOverlay_Add);

                gMapControl2.UpdateRouteLocalPosition(line_layer_Add);

                listBox2.Items.Clear();

                for (int i = 0; i < _points.Count; i++)
                {
                    listBox2.Items.Add(_points[i]);
                }
            }
        }

        // *********************************************Gelen Veri ile Karşılaştırılan verinin OTONOM için işlemdiciye gönderme kısmı**********************
        private bool otokontrollat = false;
        private bool otokontrollng = false;
        private void GPS_Flight_Plane()
        {
            if (checkBox1.Checked == true)  
            {
                
                if (o < LastValue - 1)          //Gönderilecek kordinatın indexi o değişkenidir.Burada işlemciye verileri 
                {                               //yollarken kendi indexinden fazlasını yollamayı çalışıp sistemi sıkıntya sokmasın diye koruma yapılmıştır.
                    if (_points.Count != 0)       //_points değişkeni koyduğumuz noktaların indexidir.Checkbox tikli iken eğer haritada işaretli alan yoksa sistem patlıyordu.Önlem amaçlı.  
                    {
                        if (a < 1)                  // burada ki fonksiyon sadece bizim ilk kordinatımızı yollamak için tasarlanmıştır.
                        {                           //Bunun için de a= 0 diyip 1 kere işleme sokuyoruz sonra a yı arttırıp bir daha veri göndermesi engelleniyor.
                            var otolat_bytes = new byte[8];
                            var otolng_bytes = new byte[8];

                            latot = _Lat[o];
                            lngot = _Lng[o];
                            otolat_bytes = BitConverter.GetBytes(latot);        //bytlarına ayır.
                            otolng_bytes = BitConverter.GetBytes(lngot);

                            otolat_bytes.CopyTo(otolatdizi, 37);            // yolla gitsin
                            otolng_bytes.CopyTo(otolatdizi, 45);

                            otolatdizi[0] = (byte)'F';

                            serialPort1.Write(otolatdizi, 0, otolatdizi.Length);

                            a++;
                        }
                    }
                   

                    double txtlat_buffereksi, txtlat_bufferartı, txtlng_buffereksi, txtlng_bufferartı;      //Burdaki işlem karşılaştırdığımız kordinat verilerinin hiçbir zaman eşit olmayacağı için 
                    txtlat_buffereksi =((Datas.GPS_Lat_veri * 10000000) - 1000) / 10000000;                     //burada bir kare alan oluşturuyoruz.1000 üstü ve 1000 altı aralıklarına drone girdiği anda diğer verimiz drona gidiyor.
                    txtlat_bufferartı= ((Datas.GPS_Lat_veri * 10000000) + 1000) / 10000000;         // 1000 belki biraz daha küçük olabilir latlardaa baya mesafer fark edebiliyor belki onu kü.ük yapabiliriz.
                    txtlng_buffereksi =((Datas.GPS_Lng_veri * 100000000) - 1000)/ 100000000;
                    txtlng_bufferartı =((Datas.GPS_Lng_veri * 100000000) + 1000)/ 100000000;

                    if (_Lat[o] > txtlat_buffereksi && _Lat[o] < txtlat_bufferartı)
                    {
                        otokontrollat = true;
                    }
                    else
                    {
                        otokontrollat = false;
                    }

                    if (_Lng[o] > txtlng_buffereksi && _Lng[o] < txtlng_bufferartı)
                    {
                        otokontrollng = true;
                    }
                    else
                    {
                        otokontrollng = false;
                    }

                    if (_points.Count != 0)
                    {
                        if (otokontrollat == true && otokontrollng == true)          //burada aralık kontrol ediliyor doğru ise veriler gidiyor.
                        {
                            o++;
                            var otolat_bytes = new byte[8];
                            var otolng_bytes = new byte[8];

                            latot = double.Parse(_Lat[o].ToString());           // gelen veriler latot değişkeninde saklanıyor.
                            lngot = double.Parse(_Lng[o].ToString());

                            otolat_bytes = BitConverter.GetBytes(latot);        //bytlarına ayır.
                            otolng_bytes = BitConverter.GetBytes(lngot);

                            otolat_bytes.CopyTo(otolatdizi, 37);            // yolla gitsin
                            otolng_bytes.CopyTo(otolatdizi, 45);
                            otolatdizi[0] = (byte)'F';

                            serialPort1.Write(otolatdizi, 0, otolatdizi.Length);

                        }
                    }

                   
                }



            }
        }



            // ***********************************Buradaki Fonksiyon İle Oluşturulan Rotada Geri Alma işlemi Gerçekleşiyor.*******************************

            private void gerialbtn_fonk()
        {
            
            if (_points.Count > 0)              //hiç bir veri olmaması halinde dahil butona bastığımızda bir sıkıntı çıkmaması için sadece sıfırdan büyük olduğunda silme işlemi yapıyoruz.
            {
                _points.RemoveAt(_points.Count - 1);
                
            }

            if (_Lat.Count > 0)
            {
                _Lat.RemoveAt(_Lat.Count - 1);
               
            }

            if (_Lng.Count > 0)
            {
                _Lng.RemoveAt(_Lng.Count - 1);
                
            }
            if (_points.Count == 0)
            {
                a = 0;
                o = 0;
            }


            if (routersOverlay_Add != null)
            {
                markers_add.Markers.Clear();
                routersOverlay_Add.Routes.Clear();
                gMapControl2.Overlays.Clear();
            }


            for (int i = 0; i < _points.Count; i++)             //Geri alma işlemi yaparken verilerin hepsi üstte siliniyor ardından burada geri aldığımız noktaya kadar olan kısımları geri koyuyoruz.
            {
                AddMarker(_points[i]);
            }

            line_layer_Add = new GMapRoute(_points, "single_line");
            line_layer_Add.Stroke = new Pen(Brushes.Blue, 4);
            routersOverlay_Add = new GMapOverlay("routes");
            routersOverlay_Add.Routes.Add(line_layer_Add);
            gMapControl2.Overlays.Add(routersOverlay_Add);

            //To force the draw, you need to update the route
            gMapControl2.UpdateRouteLocalPosition(line_layer_Add);

            gMapControl2.Refresh();

            listBox2.Items.Clear();

            for (int i = 0; i < _points.Count; i++)
            {
                listBox2.Items.Add(_points[i]);
            }
            gerialbtn.Enabled = true;           // iki kere arka arkaya butona basıp sıkıntı yaşamamak için alınan önlem.
            
        }
        
        private void gerialbtn_Click(object sender, EventArgs e)
        {
            
            th4_gerial = new Thread(gerialbtn_fonk);        //thread çalıştırılıyor.
            th4_gerial.Start();
            gerialbtn.Enabled = false;

        }

        //*********************************Temizle Butonu ile harita üstündeki Tüm markerler Temizlenir.***************************************

        private void temizlebtn_Click(object sender, EventArgs e)
        {
            _points.Clear();
            _Lat.Clear();                   
            _Lng.Clear();
            o = 0;
            a = 0;
            if (routersOverlay_Add != null)
            {
                markers_add.Markers.Clear();
                routersOverlay_Add.Routes.Clear();
                gMapControl2.Overlays.Clear();
            }
           
            gMapControl2.Refresh();

            listBox2.Items.Clear();

            for (int i = 0; i < _points.Count; i++)
            {
                listBox2.Items.Add(_points[i]);
            }
        }
        
        public void DiscardOutBuffer()                                  //Seri portu kapatırken bufferın içini boşaltıyoruz.
        {
            serialPort1.DiscardOutBuffer();
        }

      
        public void DiscardInBuffer()            //Seri portu açarken bufferın içini boşaltıyoruz.
        {                                        //Bazen haberleşme esnasında kullanmadığınız ama portda kalan verileriniz olabilir böyle bir durumda ikinci kez 
                                                 //veri okumaya kalktığınız da hem o önceki okumada kalan verileri hem de cihazdan gelen doğru cevabı alırsınız

            serialPort1.DiscardInBuffer();

        }
        private bool _scrolling = true;
        private void button1_Click(object sender, EventArgs e)
        {
            
                if (!_scrolling)                            //Grafikleri durdurmak için kullandığımız fonksiyon.
                {
                    button1.Text = @"Pause Scrolling";
                    _scrolling = true;
                }
                else
                {
                    button1.Text = @"Resume Scrolling";
                    _scrolling = false;
                }
            
        }

        

        private void zedGraphControl1_MouseClick(object sender, MouseEventArgs e)
        {
           
        }

        private void button2_Click(object sender, EventArgs e)
        {

            float Drone_Komut;
            Drone_Komut = float.Parse(txt_dronekomut.Text);    //verileri virgüllü göndermek için parseliyoruz.
            

            if (serialPort1.IsOpen)
            {
                var Drone_Komut_bytes = new byte[4];

                Drone_Komut_bytes = BitConverter.GetBytes(Drone_Komut);


                Drone_Komut_bytes.CopyTo(Dronekomut_dizi, 53);    // Verileri mydizinin içine atıyoruz seriporttan göndermek için.

                Dronekomut_dizi[0] = (byte)'D';

                serialPort1.Write(Dronekomut_dizi, 0, Dronekomut_dizi.Length);

            }
        }
        int sayac = 0;
        private void timer1_Tick(object sender, EventArgs e)
        {
            if (sayac == 0)
            {
                txt_errorflag.BackColor = Color.Red; sayac++;
                picturebx_error.BackColor = Color.Blue;
            }
            else
            {
                txt_errorflag.BackColor = Color.DarkRed; sayac--;
                picturebx_error.BackColor = Color.White;
            }
        }
        
        
        private void pitch_angle_text_TextChanged(object sender, EventArgs e)
        {
            
        }
        
        

        private void zedGraphControl2_ZoomEvent(ZedGraphControl sender, ZoomState oldState, ZoomState newState)
        {
            myPane.YAxis.Scale.Min = myPane2.YAxis.Scale.Min;
            myPane.YAxis.Scale.Max = myPane2.YAxis.Scale.Max;

            myPane.XAxis.Scale.Min = myPane2.XAxis.Scale.Min;
            myPane.XAxis.Scale.Max = myPane2.XAxis.Scale.Max;


            myPane3.YAxis.Scale.Min = myPane2.YAxis.Scale.Min;
            myPane3.YAxis.Scale.Max = myPane2.YAxis.Scale.Max;

            myPane3.XAxis.Scale.Min = myPane2.XAxis.Scale.Min;
            myPane3.XAxis.Scale.Max = myPane2.XAxis.Scale.Max;


            myPane4.YAxis.Scale.Min = myPane2.YAxis.Scale.Min;
            myPane4.YAxis.Scale.Max = myPane2.YAxis.Scale.Max;

            myPane4.XAxis.Scale.Min = myPane2.XAxis.Scale.Min;
            myPane4.XAxis.Scale.Max = myPane2.XAxis.Scale.Max;



            zedGraphControl1.AxisChange();
            zedGraphControl2.AxisChange();
            zedGraphControl3.AxisChange();
            zedGraphControl4.AxisChange();


            myPane.AxisChange();
            zedGraphControl1.Refresh();


            myPane2.AxisChange();
            zedGraphControl2.Refresh();

            myPane3.AxisChange();
            zedGraphControl3.Refresh();

            myPane4.AxisChange();
            zedGraphControl4.Refresh();
        }

        private void zedGraphControl3_ZoomEvent(ZedGraphControl sender, ZoomState oldState, ZoomState newState)
        {
            myPane2.YAxis.Scale.Min = myPane3.YAxis.Scale.Min;
            myPane2.YAxis.Scale.Max = myPane3.YAxis.Scale.Max;

            myPane2.XAxis.Scale.Min = myPane3.XAxis.Scale.Min;
            myPane2.XAxis.Scale.Max = myPane3.XAxis.Scale.Max;


            myPane.YAxis.Scale.Min = myPane3.YAxis.Scale.Min;
            myPane.YAxis.Scale.Max = myPane3.YAxis.Scale.Max;

            myPane.XAxis.Scale.Min = myPane3.XAxis.Scale.Min;
            myPane.XAxis.Scale.Max = myPane3.XAxis.Scale.Max;


            myPane4.YAxis.Scale.Min = myPane3.YAxis.Scale.Min;
            myPane4.YAxis.Scale.Max = myPane3.YAxis.Scale.Max;

            myPane4.XAxis.Scale.Min = myPane3.XAxis.Scale.Min;
            myPane4.XAxis.Scale.Max = myPane3.XAxis.Scale.Max;



            zedGraphControl1.AxisChange();
            zedGraphControl2.AxisChange();
            zedGraphControl3.AxisChange();
            zedGraphControl4.AxisChange();


            myPane.AxisChange();
            zedGraphControl1.Refresh();


            myPane2.AxisChange();
            zedGraphControl2.Refresh();

            myPane3.AxisChange();
            zedGraphControl3.Refresh();

            myPane4.AxisChange();
            zedGraphControl4.Refresh();
        }

        private void zedGraphControl4_ZoomEvent(ZedGraphControl sender, ZoomState oldState, ZoomState newState)
        {
            myPane2.YAxis.Scale.Min = myPane4.YAxis.Scale.Min;
            myPane2.YAxis.Scale.Max = myPane4.YAxis.Scale.Max;

            myPane2.XAxis.Scale.Min = myPane4.XAxis.Scale.Min;
            myPane2.XAxis.Scale.Max = myPane4.XAxis.Scale.Max;


            myPane3.YAxis.Scale.Min = myPane4.YAxis.Scale.Min;
            myPane3.YAxis.Scale.Max = myPane4.YAxis.Scale.Max;

            myPane3.XAxis.Scale.Min = myPane4.XAxis.Scale.Min;
            myPane3.XAxis.Scale.Max = myPane4.XAxis.Scale.Max;


            myPane.YAxis.Scale.Min = myPane4.YAxis.Scale.Min;
            myPane.YAxis.Scale.Max = myPane4.YAxis.Scale.Max;

            myPane.XAxis.Scale.Min = myPane4.XAxis.Scale.Min;
            myPane.XAxis.Scale.Max = myPane4.XAxis.Scale.Max;



            zedGraphControl1.AxisChange();
            zedGraphControl2.AxisChange();
            zedGraphControl3.AxisChange();
            zedGraphControl4.AxisChange();


            myPane.AxisChange();
            zedGraphControl1.Refresh();


            myPane2.AxisChange();
            zedGraphControl2.Refresh();

            myPane3.AxisChange();
            zedGraphControl3.Refresh();

            myPane4.AxisChange();
            zedGraphControl4.Refresh();
        }

       
        PointF centrePoint = new System.Drawing.PointF();
        private void zedGraphControl1_Scroll(object sender, ScrollEventArgs e)
        {

        }
        

        private void zedGraphControl1_ScrollEvent(object sender, ScrollEventArgs e)
        {

          
        }
        
        private void zedGraphControl1_ZoomEvent(ZedGraphControl sender, ZoomState oldState, ZoomState newState)
        {
            myPane2.YAxis.Scale.Min = myPane.YAxis.Scale.Min;
            myPane2.YAxis.Scale.Max = myPane.YAxis.Scale.Max;

            myPane2.XAxis.Scale.Min = myPane.XAxis.Scale.Min;
            myPane2.XAxis.Scale.Max = myPane.XAxis.Scale.Max;


            myPane3.YAxis.Scale.Min = myPane.YAxis.Scale.Min;
            myPane3.YAxis.Scale.Max = myPane.YAxis.Scale.Max;

            myPane3.XAxis.Scale.Min = myPane.XAxis.Scale.Min;
            myPane3.XAxis.Scale.Max = myPane.XAxis.Scale.Max;


            myPane4.YAxis.Scale.Min = myPane.YAxis.Scale.Min;
            myPane4.YAxis.Scale.Max = myPane.YAxis.Scale.Max;

            myPane4.XAxis.Scale.Min = myPane.XAxis.Scale.Min;
            myPane4.XAxis.Scale.Max = myPane.XAxis.Scale.Max;



            zedGraphControl1.AxisChange();
            zedGraphControl2.AxisChange();
            zedGraphControl3.AxisChange();
            zedGraphControl4.AxisChange();


            myPane.AxisChange();
            zedGraphControl1.Refresh();


            myPane2.AxisChange();
            zedGraphControl2.Refresh();

            myPane3.AxisChange();
            zedGraphControl3.Refresh();

            myPane4.AxisChange();
            zedGraphControl4.Refresh();
        }

        private void headingIndicator1_Load(object sender, EventArgs e)
        {

        }
    }
}

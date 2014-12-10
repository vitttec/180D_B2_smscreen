using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
//using Contr

namespace BluetoothController
{
    public class VirtualSensorData : IData
    {
        public double[] acceleration;
        public double[] velocity;
        public double[] position;
        public double[] thequat;
        public bool isinferredornottracked;
        public long NowInTicks { get;  set; }
        public VirtualSensorData()
        {
            this.acceleration = new double[3];
            this.velocity = new double[3];
            this.position = new double[3];
            this.thequat = new double[4];
            this.isinferredornottracked = false;
            this.NowInTicks = DateTime.UtcNow.Ticks;
        }
        public VirtualSensorData(VirtualSensorData v)
        {
            this.isinferredornottracked = v.isinferredornottracked;
            this.NowInTicks = v.NowInTicks;

            this.acceleration = new double[v.acceleration.Length];
            for (int i = 0; i < v.acceleration.Length; i++)
                this.acceleration[i] = v.acceleration[i];

            this.velocity = new double[v.velocity.Length];
            for (int i = 0; i < v.velocity.Length; i++)
                this.velocity[i] = v.velocity[i];

            this.position = new double[v.position.Length];
            for (int i = 0; i < v.position.Length; i++)
                this.position[i] = v.position[i];

            this.thequat = new double[v.thequat.Length];
            for (int i = 0; i < v.thequat.Length; i++)
                this.thequat[i] = v.thequat[i];
        }

        #region IData Members
        public string ToFileHeaderString()
        {
            StringBuilder sb = new StringBuilder();
            /*
            sb.AppendFormat("{0}, {1}, {2}, {3}{4}",
                    "Time",
                    "Ax",
                    "Ay",
                    "Az",
                    Environment.NewLine);
            */
            sb.AppendFormat("{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}",
                "Time",
                "Ax",
                "Ay",
                "Az",
                "QW",
                "QX",
                "QY",
                "QZ",
                Environment.NewLine);
             
            return sb.ToString();
        }
       
        public string ToFileString()
        {
            StringBuilder sb = new StringBuilder();
            
            
            sb.AppendFormat("{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}",
                this.NowInTicks,
                this.acceleration[0],
                this.acceleration[1],
                this.acceleration[2],
                this.thequat[0],
                this.thequat[1],
                this.thequat[2],
                this.thequat[3]
                /*             
                Global.theQuat.W,
                Global.theQuat.X,
                Global.theQuat.Y,
                Global.theQuat.Z
                */
                );
            
            sb.Append(Environment.NewLine);
            return sb.ToString();
        }
        public string ToPreviewString()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendFormat("vx:{0:D},vy:{1:D},vz{2:D}",
                velocity[0].ToString("+#.###;-#.###;0.000"),
                velocity[1].ToString("+#.###;-#.###;0.000"),
                velocity[2].ToString("+#.###;-#.###;0.000"));
            sb.AppendFormat("ax:{0:D},ay:{1:D},az{2:D}",
                acceleration[0].ToString("f"),
                acceleration[1].ToString("f"),
                acceleration[2].ToString("f"));
            sb.AppendFormat("qw:{0}, qx:{2}, qy: {2}, qz: {3}",
                thequat[0].ToString("f"), thequat[1].ToString("f"), thequat[2].ToString("f"), thequat[3].ToString("f"));
            return sb.ToString();
        }
        public string ToWindowsFormString()
        {
            throw new NotImplementedException();
        }
        #endregion
    }
}

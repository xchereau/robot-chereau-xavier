using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RobotInterface
{
    public class Robot
    {
        public string receivedText = "";
        public float distanceTelemetreDroit;
        public float distanceTelemetreCentre;
        public float distanceTelemetreGauche;
        public float positionXodometry;
        public float positionYodometry;
        public float timestamp;
        public float angle_radian;
        public float vitesselineaire;
        public float vitesseangulaire;

        public Queue<byte> byteListReceived = new Queue<byte>();

        public Robot()
        {
            
        }
    }
}
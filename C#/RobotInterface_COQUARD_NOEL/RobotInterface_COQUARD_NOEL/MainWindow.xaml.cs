﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using ExtendedSerialPort;
using System.IO.Ports;
using System.Windows.Threading;

#pragma warning disable CS8618 // Un champ non-nullable doit contenir une valeur non-null lors de la fermeture du constructeur. Envisagez de déclarer le champ comme nullable.

namespace RobotInterface_COQUARD_NOEL
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 
    
    public partial class MainWindow : Window
    {
        bool etatBouton;
        bool retour;
        ReliableSerialPort serialPort1;
        DispatcherTimer timerAffichage;
        Robot robot = new Robot();

        public object SerialPort1 { get; private set; }

      public MainWindow()
     {
            timerAffichage = new DispatcherTimer();
            timerAffichage.Interval = new TimeSpan(0, 0, 0, 0, 100);
            timerAffichage.Tick += TimerAffichage_Tick;
            timerAffichage.Start();

            serialPort1 = new ExtendedSerialPort.ReliableSerialPort("COM5", 115200, Parity.None, 8, StopBits.One);
            serialPort1.OnDataReceivedEvent += SerialPort1_DataReceived; 
            serialPort1.Open();
            InitializeComponent();
            etatBouton = true; 
            retour = false;
            robot.receivedText = "";
        }

        private void TimerAffichage_Tick(object? sender, EventArgs e)
        {
            if (robot.receivedText != "")
            {
                textBoxReception.Text = "Reçu Port : " + robot.receivedText ;
                robot.receivedText = "";
            }
            //throw new NotImplementedException();
        }

        public void SerialPort1_DataReceived(object? sender, DataReceivedArgs e)
        {
            robot.receivedText += Encoding.UTF8.GetString(e.Data, 0, e.Data.Length);
        }

        private void textBoxEmission_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void textBoxReception_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

        private void buttonEnvoyer_Click(object sender, RoutedEventArgs e)
        {
            etatBouton = !etatBouton ;
            retour = true;
            if (etatBouton == false)
            {
                buttonEnvoyer.Background = Brushes.RoyalBlue;
            }
            else
            {
                buttonEnvoyer.Background = Brushes.Beige;
            }
            SendMessage(retour);
        }

        private void textBoxEmission_KeyUp(object sender, KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                retour = false;
                SendMessage(retour);
            }
            if (e.Key == Key.Delete)
            {
                textBoxReception.Clear();
            }
        }

        private void SendMessage(bool retour)
        {
            if (retour)
            {
                textBoxEmission.Text += "\r\n";
            }
            textBoxReception.Text += "Reçu : " + textBoxEmission.Text ;
            serialPort1.WriteLine(textBoxEmission.Text);
            textBoxEmission.Clear();
        }

        private void buttonClear_Click(object sender, RoutedEventArgs e)
        {
            textBoxReception.Clear();
        }
    }
}
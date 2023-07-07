using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;

using System.Threading;

using ROBOTIS;

using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.UI;

using System.Diagnostics;

namespace Emgu1
{
    public partial class Form1 : Form
    {
        //VARIABLES PARA LOS HILOS

        //Crear hilos para la camara y la comunicación zigbee
        Thread hilo_camara;
        Thread hilo_zigbee;

        //Crear enteros para conocer el estado de los hilos (En Ejecución, Suspendido, etc)
        int hilo_camara_estado = 0;
        int hilo_zigbee_estado = 0;

        //Crear delegados para poder modificar los pictureBox y label desde el hilo de la cámara
        public delegate void cambiarImagen(Image<Hsv, byte> imagen1, Image<Bgr, Byte> imagen2, Image<Gray, Byte> imagen3, Image<Gray, Byte> imagen4);
        public delegate void cambiarTexto(String texto1, String texto2, String texto3, String texto4);


        /////////////////////////////////////////////////////////////////////////////////////////////////


        //VARIABLES PARA LOS BIOLOID
        public const int DEFAULT_PORTNUM = 4; // COM4, el original aquí era COM3
        public const int TIMEOUT_TIME = 1000; // msec

        int TxData, RxData;
        int i;
        int numero = 0;
        bool click = false;


        //////////////////////////////////////////////////////////////////////////////////////////////////


        //VARIABLES PARA LA CÁMARA       

        //Variable para capturar video de la webcam
        private Capture capturar = new Capture();

        //Variables para capturar imágenes a partir del video y posteriormente extraer la propiedad Data
        Image<Bgr, byte> img = new Image<Bgr, byte>(640, 480);
        Image<Hsv, byte> img2 = new Image<Hsv, byte>(640, 480);        

        //Imagenes para Escala de Grises, Filtrado Canny y Detección de figuras
        Image<Gray, Byte> gray = new Image<Gray, Byte>(640, 480);
        Image<Gray, Byte> cannyEdges = new Image<Gray, Byte>(640, 480);
        Image<Gray, Byte> cannyEdges2 = new Image<Gray, Byte>(640, 480);
        Image<Bgr, Byte> triangleRectangleImage = new Image<Bgr, Byte>(640, 480);

        //Variables para guardar la propiedad Data de las imágenes
        byte[, ,] datos = new byte[640, 480, 3];

        //Variables para el filtrado Canny
        Gray cannyThreshold = new Gray(250); //original era 180, nuevo reciente era 150
        Gray cannyThresholdLinking = new Gray(20); //original era 120, nuevo reciente era 2

        //Bandera para encender/apagar webcam
        bool estado = true;

        //Variables para calcular Frames Per Second (FPS)
        Stopwatch reloj = new Stopwatch();
        TimeSpan ts = new TimeSpan();
        double msecs = 0;
        double fps = 0;


        public Form1()
        {
            InitializeComponent();

            //Definir resolución a la cual capturo video (640x480, relación 4:3)
            capturar.SetCaptureProperty(Emgu.CV.CvEnum.CAP_PROP.CV_CAP_PROP_FRAME_WIDTH, 640);   
            capturar.SetCaptureProperty(Emgu.CV.CvEnum.CAP_PROP.CV_CAP_PROP_FRAME_HEIGHT, 480);

            //Crear los hilos con sus funciones correspondientes
            hilo_camara = new Thread(new ThreadStart(mostrar));
            hilo_zigbee = new Thread(new ThreadStart(comunicar));
 
        }

        private void modificar_pictureBox(Image<Hsv, byte> imagen1, Image<Bgr, Byte> imagen2, Image<Gray, Byte> imagen3, Image<Gray, Byte> imagen4)
        {
            pictureBox1.Image = imagen1.ToBitmap();
            pictureBox2.Image = imagen2.ToBitmap();
            pictureBox3.Image = imagen3.ToBitmap();
            pictureBox4.Image = imagen4.ToBitmap();
        }

        private void modificar_label(String texto1, String texto2, String texto3, String texto4)
        {
            label1.Text = texto1;
            label2.Text = texto2;
            label3.Text = texto3;
            label4.Text = texto4;
        }

        private void comunicar()
        {
            while (true)
            {
                if (click == true)
                {
                    //Enviar dato
                    TxData = numero;
                    zigbee.zgb_tx_data(TxData);

                    //Recibir dato
                    for (i = 0; i < TIMEOUT_TIME; i++)
                    {
                        // Verify data recieved
                        if (zigbee.zgb_rx_check() == 1)
                        {
                            // Ver Dato recibido
                            RxData = zigbee.zgb_rx_data();
                            break;
                        }
                    }   

                    //Si los datos enviados y recibidos son iguales, validar mensaje
                    if (RxData == TxData)
                    {
                        click = false;
                    }
                }  
                Thread.Sleep(50);
            }
        }

        private void mostrar()
        {
            while (true)
            { 
                //Encender reloj para FPS
                reloj.Start();

                //Capturar un frame, convertir a Hsv y mostrar en pantalla
                img = capturar.QueryFrame();
                img2 = img.Convert<Hsv, Byte>();

                //Extraer todos los datos de la imagen Hsv
                datos = img2.Data;

                //Convertir la imagen a Escala de Grises y filtrar el ruido
                gray = img.Convert<Gray, Byte>().PyrDown().PyrUp();

                //Resetear los booleanos de sombreros detectados
                bool esRojo = false;
                bool esNaranja = false;
                bool esAmarillo = false;
                bool esVerde = false;
                bool esAzul = false;


                #region Filtro Canny y detección de bordes
                cannyEdges = gray.Canny(cannyThreshold, cannyThresholdLinking);
                cannyEdges = cannyEdges.Dilate(1);
                cannyEdges2 = gray.Canny(cannyThreshold, cannyThresholdLinking);
                cannyEdges2 = cannyEdges2.Dilate(14);
                #endregion


                #region Encontrar triángulos y rectángulos
                List<Triangle2DF> triangleList = new List<Triangle2DF>();
                List<Triangle2DF> triangleListMesa = new List<Triangle2DF>();
                List<MCvBox2D> boxList = new List<MCvBox2D>(); //Una box es un rectángulo rotado

                //Booleano para detectar una única mesa triangular
                bool isTriangleMesa = false;


                using (MemStorage storage = new MemStorage()) //Allocate storage for contour approximation
                    for (
                       Contour<Point> contours = cannyEdges.FindContours(
                          Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_SIMPLE,
                          Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_LIST,
                          storage);
                       contours != null;
                       contours = contours.HNext)
                    {
                        Contour<Point> currentContour = contours.ApproxPoly(contours.Perimeter * 0.05, storage);

                        if (currentContour.Area < 1000 && currentContour.Total == 3) //Original era 250, nuevo reciente fue 2000. Contorno tiene 3 vertices, son sombreros de los robots
                        {
                            bool isTriangle = false;
                            Point[] pts = currentContour.ToArray();
                            Triangle2DF triangulo = new Triangle2DF(
                                pts[0],
                                pts[1],
                                pts[2]
                                );

                            //Hallar coordenadas del centroide
                            int x = (int)triangulo.Centeroid.X;
                            int y = (int)triangulo.Centeroid.Y;

                            #region detectar y diferenciar 5 triangulos según su color

                            //Buscar al triangulo rojo
                            if (datos[y, x, 0] > 121 && esRojo == false)
                            {
                                esRojo = true;
                                isTriangle = true;
                            }
                            //Buscar al triangulo naranja
                            else if (datos[y, x, 0] <= 20 && esNaranja == false)
                            {
                                esNaranja = true;
                                isTriangle = true;
                            }
                            //Buscar al triangulo amarillo
                            else if (datos[y, x, 0] > 21 && datos[y, x, 0] <= 35 && esAmarillo == false)
                            {
                                esAmarillo = true;
                                isTriangle = true;
                            }
                            //Buscar al triangulo verde
                            else if (datos[y, x, 0] > 35 && datos[y, x, 0] <= 50 && esVerde == false)
                            {
                                esVerde = true;
                                isTriangle = true;
                            }
                            //Buscar al triangulo azul
                            else if (datos[y, x, 0] >= 81 && datos[y, x, 0] < 100 && esAzul == false)
                            {
                                esAzul = true;
                                isTriangle = true;
                            }

                            #endregion

                            if (isTriangle) triangleList.Add(triangulo);
                        }
                        else if (currentContour.Area > 2000 && isTriangleMesa == false && currentContour.Total == 3) //Detectar la mesa triangular
                        {
                            isTriangleMesa = true;
                            Point[] pts = currentContour.ToArray();
                            triangleListMesa.Add(new Triangle2DF(
                                pts[0],
                                pts[1],
                                pts[2]
                                ));
                        }

                    }
                using (MemStorage storage = new MemStorage()) //Allocate storage for contour approximation
                    for (
                       Contour<Point> contours2 = cannyEdges2.FindContours(
                          Emgu.CV.CvEnum.CHAIN_APPROX_METHOD.CV_CHAIN_APPROX_SIMPLE,
                          Emgu.CV.CvEnum.RETR_TYPE.CV_RETR_LIST,
                          storage);
                       contours2 != null;
                       contours2 = contours2.HNext)
                    {
                        Contour<Point> currentContour = contours2.ApproxPoly(contours2.Perimeter * 0.05, storage);
                        if (currentContour.Area > 1000 && currentContour.Total == 4) //Original era 250, nuevo reciente fue 2000. Contorno tiene 4 vertices, son las mesas tipo caja
                        {
                            #region determinar si todos los ángulos en el contorno están dentro del rango [80, 100]

                            //Booleano para determinar si es rectángulo
                            bool isRectangle = true;

                            Point[] pts = currentContour.ToArray();
                            LineSegment2D[] edges = PointCollection.PolyLine(pts, true);

                            for (int i = 0; i < edges.Length; i++)
                            {
                                double angle = Math.Abs(
                                   edges[(i + 1) % edges.Length].GetExteriorAngleDegree(edges[i]));
                                if (angle < 80 || angle > 100)
                                {
                                    isRectangle = false;
                                    break;
                                }
                            }
                            #endregion

                            if (isRectangle)
                            {
                                MCvBox2D caja = currentContour.GetMinAreaRect();
                                boxList.Add(caja);
                            }
                        }
                    }
                #endregion


                #region Preparar dibujado de triángulos y rectángulos
                triangleRectangleImage = img.CopyBlank();
                foreach (Triangle2DF triangle in triangleList)
                {
                    //Obtener coordenadas X, Y del triángulo actual
                    int x = (int)triangle.Centeroid.X;
                    int y = (int)triangle.Centeroid.Y;

                    #region Buscar y dibujar los 5 triángulos

                    //Buscar al triangulo rojo
                    if (datos[y, x, 0] > 121)
                    {
                        triangleRectangleImage.Draw(triangle, new Bgr(Color.Red), 2);

                        this.BeginInvoke(new cambiarTexto(modificar_label), new object[] { x.ToString(), y.ToString(), msecs.ToString(), fps.ToString() });
                    }
                    //Buscar al triangulo naranja
                    else if (datos[y, x, 0] <= 20)
                    {
                        triangleRectangleImage.Draw(triangle, new Bgr(Color.DarkOrange), 2);
                    }
                    //Buscar al triangulo amarillo
                    else if (datos[y, x, 0] > 21 && datos[y, x, 0] <= 35)
                    {
                        triangleRectangleImage.Draw(triangle, new Bgr(Color.Yellow), 2);

                    }
                    //Buscar al triangulo verde
                    else if (datos[y, x, 0] > 35 && datos[y, x, 0] <= 50)
                    {
                        triangleRectangleImage.Draw(triangle, new Bgr(Color.Green), 2);
                    }
                    //Buscar al triangulo azul
                    else if (datos[y, x, 0] >= 81 && datos[y, x, 0] < 100)
                    {
                        triangleRectangleImage.Draw(triangle, new Bgr(Color.Blue), 2);
                    }

                    #endregion
                }
                foreach (Triangle2DF triangle in triangleListMesa)
                {
                    triangleRectangleImage.Draw(triangle, new Bgr(Color.White), 2);
                }
                foreach (MCvBox2D box in boxList)
                    triangleRectangleImage.Draw(box, new Bgr(Color.White), 2);


                #endregion

                //Enviar imágenes al delegado para que las coloque en los pictureBox correspondientes
                this.BeginInvoke(new cambiarImagen(modificar_pictureBox), new object[] { img2, triangleRectangleImage, cannyEdges, cannyEdges2 });

                //Calcular FPS
                ts = reloj.Elapsed;
                msecs = ts.Milliseconds;
                fps = 1 / (msecs / 1000);

                //resetear reloj para FPS
                reloj.Reset();
            }

            
        }

        private void button1_Click(object sender, EventArgs e)
        {
            if (capturar != null)
            {
                if (estado)
                {
                    //Apagar cámara
                    button1.Text = "Detener";
                    
                    //Iniciar o resumir el hilo de la cámara, segun el caso
                    if (hilo_camara_estado == 0) { hilo_camara.Start(); }
                    else if (hilo_camara_estado == 1) { hilo_camara.Resume(); }

                    //Apagar y resetear reloj para FPS
                    reloj.Stop();
                    reloj.Reset();
                }
                else
                {
                    //Encender cámara 
                    button1.Text = "Empezar";

                    //Suspender el hilo de la cámara
                    hilo_camara.Suspend();
                    hilo_camara_estado = 1;

                    //Encender reloj para FPS
                    reloj.Start();
                }
                estado = !estado;
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            // Open device
            if (zigbee.zgb_initialize(DEFAULT_PORTNUM) == 0)
            {
                label5.Text = "Failed to open Zig2Serial!";
            }
            else
                label5.Text = "Succeed to open Zig2Serial!";

            //Iniciar o resumir el hilo de la cámara, segun el caso
            if (hilo_zigbee_estado == 0) { hilo_zigbee.Start(); }
            else if (hilo_zigbee_estado == 1) { hilo_zigbee.Resume(); }
        }

        private void button3_Click(object sender, EventArgs e)
        {
            // Close device
            zigbee.zgb_terminate();

            //Suspender el hilo de zigbee
            hilo_zigbee.Suspend();
            hilo_zigbee_estado = 1;  
        }

        private void button4_Click(object sender, EventArgs e)
        {
            //Sentado
            numero = 1;
            click = true;
        }

        private void button5_Click(object sender, EventArgs e)
        {
            //Caminar hacia adelante
            numero = 6;
            click = true;
        }

        private void button6_Click(object sender, EventArgs e)
        {
            //De pie
            numero = 2;
            click = true;
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            //Apagar cámara y descartar datos de video guardados
            capturar.Dispose();
        }

        private void button7_Click(object sender, EventArgs e)
        {
            //Terminar todos los hilos
            hilo_camara.Abort();
            hilo_zigbee.Abort();
        }

    }
}

using System.Runtime.InteropServices;

namespace ROBOTIS
{
    unsafe class zigbee
    {
        public const int RC100_BTN_U = 1;
        public const int RC100_BTN_D = 2;
        public const int RC100_BTN_L = 4;
        public const int RC100_BTN_R = 8;
        public const int RC100_BTN_1 = 16;
        public const int RC100_BTN_2 = 32;
        public const int RC100_BTN_3 = 64;
        public const int RC100_BTN_4 = 128;
        public const int RC100_BTN_5 = 256;
        public const int RC100_BTN_6 = 512;

        [DllImport("zigbee.dll")]
        public static extern int zgb_initialize(int devIndex);

        [DllImport("zigbee.dll")]
        public static extern void zgb_terminate();

        [DllImport("zigbee.dll")]
        public static extern int zgb_tx_data(int data);

        [DllImport("zigbee.dll")]
        public static extern int zgb_rx_check();

        [DllImport("zigbee.dll")]
        public static extern int zgb_rx_data();
    }    
}
#include <gtk/gtk.h>
#include <math.h>
#include <cairo.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>

int fd;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
double acclX_scaled, acclY_scaled, acclZ_scaled;
double gyroX_scaled, gyroY_scaled, gyroZ_scaled;
gboolean updateLevel(gpointer);

int read_word_2c(int addr)
{
  int val;
  val = wiringPiI2CReadReg8(fd, addr);
  val = val << 8;
  val += wiringPiI2CReadReg8(fd, addr+1);
  if (val >= 0x8000)
  val = -(65536 - val);
   
  return val;
}
 
double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}
 
double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}
 
double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}

void closeMainWindow (GtkButton* button, gpointer user_data)
{
  gtk_main_quit();
}

gboolean updateLevel(gpointer data)
{
    GtkWidget *levelValue = data;

    acclX = read_word_2c(0x3B);
    acclY = read_word_2c(0x3D);
    acclZ = read_word_2c(0x3F);
     
    acclX_scaled = acclX / 16384.0;
    acclY_scaled = acclY / 16384.0;
    acclZ_scaled = acclZ / 16384.0;
     
    printf("My acclX_scaled: %f\n", acclX_scaled);
    printf("My acclY_scaled: %f\n", acclY_scaled);
    printf("My acclZ_scaled: %f\n", acclZ_scaled);
     
    printf("My X rotation: %f\n", get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled));
    printf("My Y rotation: %f\n", get_y_rotation(acclX_scaled, acclY_scaled, acclZ_scaled));    

    gchar tmpbuf[100];
    sprintf(tmpbuf , "%.2f\n", get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled));
    gtk_label_set_text (GTK_LABEL(levelValue), tmpbuf);

   return TRUE;
}

int main(int argc, char *argv[])
{
  GtkWidget *window, *rotationCircle;
  GtkButton *buttonClose;
  GtkBuilder *builder;
  GtkLabel *levelValue;

  gtk_init (&argc, &argv);

  builder = gtk_builder_new ();
  gtk_builder_add_from_file (builder, "Main.glade", NULL);

  window = GTK_WIDGET(gtk_builder_get_object (builder, "mainWindow"));
  buttonClose = GTK_BUTTON(gtk_builder_get_object (builder, "buttonClose"));
  levelValue = GTK_LABEL(gtk_builder_get_object (builder, "levelValue"));

  gtk_builder_connect_signals (builder, NULL);
  gtk_window_maximize (GTK_WINDOW(window));
  gtk_widget_show (window);

  fd = wiringPiI2CSetup (0x68);
  wiringPiI2CWriteReg8 (fd,0x6B,0x00);
  printf("set 0x6B=%X\n",wiringPiI2CReadReg8 (fd,0x6B));

  g_timeout_add(1000, updateLevel, levelValue);

  gtk_main ();      
  return 0;
}
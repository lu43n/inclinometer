#include <gtk/gtk.h>
#include <math.h>
#include <cairo.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include "kalman.h"

int fd;
int temperature;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
float acclX_scaled, acclY_scaled, acclZ_scaled, xRotation, yRotation;
float gyroX_scaled, gyroY_scaled, gyroZ_scaled, temperatureDegrees;
float xRotationComplementary, yRotationComplementary;

GtkLabel *levelValue, *rotationValue;
guint width, height;
GtkStyleContext *context;
GtkWidget *window, *rotationCircle;
GtkButton *buttonClose;
GtkBuilder *builder;
cairo_t *cr;

time_t timer;
float dt;
Kalman *kalmanFilter;

gdouble rotation = 0;

void closeMainWindow (GtkButton* button, gpointer user_data)
{
  gtk_main_quit();
}

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

gboolean drawCircle (GtkWidget *widget, cairo_t *cr, gpointer data)
{
    int radius, p1x, p1xn, p1y, p1yn, p2x, p2xn, p2y, p2yn, cx, cy;
    context = gtk_widget_get_style_context (rotationCircle);
    width = gtk_widget_get_allocated_width (rotationCircle);
    height = gtk_widget_get_allocated_height (rotationCircle);

    cairo_set_source_rgba (cr, 0, 0, 0, 0.3);
    cairo_set_line_width (cr, 5);

    if (width < height)
    {
      radius = width/2 - 4;
    }
    else
    {
      radius = height/2 - 4;
    }

    cairo_arc (cr, width / 2, height / 2, radius, 0, 2 * M_PI);
    cairo_stroke_preserve(cr); 

    cairo_set_source_rgba (cr, 255, 255, 255, 0.6);
    cairo_fill (cr); 

    cairo_set_source_rgba (cr, 1, 0.2, 0.2, 0.6);
    cairo_set_line_width (cr, 6.0);

    cairo_move_to (cr, width / 2, height / 2);
    cairo_line_to (cr, (width / 2) + radius * cos(rotation), (height / 2) + radius * sin(rotation));

    cairo_move_to (cr, width / 2, height / 2);
    cairo_line_to (cr, ((width / 2) - radius * cos(rotation)), (height / 2) - radius * sin(rotation));

    cairo_stroke (cr);

    return FALSE;
}

gboolean updateRotation(GtkWidget *widget)
{
  rotation = yRotation;

  gtk_widget_queue_draw(widget);

   return TRUE;
}

gboolean updateReadings(GtkWidget *widget)
{
    gchar level[100], rotation[100];

    sprintf(level, "%.1f°", xRotation);
    gtk_label_set_text (GTK_LABEL(levelValue), level)
    ;
    sprintf(rotation, "%.1f°", yRotation);
    gtk_label_set_text (GTK_LABEL(rotationValue), rotation);

   return TRUE;
}

gboolean updateSensorData(GtkWidget *widget)
{
    gyroX = read_word_2c(0x43);
    gyroY = read_word_2c(0x45);
    gyroZ = read_word_2c(0x47);

    gyroX_scaled = gyroX / 131.0;
    gyroY_scaled = gyroY / 131.0;
    gyroZ_scaled = gyroZ / 131.0;

    acclX = read_word_2c(0x3B);
    acclY = read_word_2c(0x3D);
    acclZ = read_word_2c(0x3F);
     
    acclX_scaled = acclX / 16384.0;
    acclY_scaled = acclY / 16384.0;
    acclZ_scaled = acclZ / 16384.0;

    temperature = read_word_2c(0x41);
    temperatureDegrees = (temperature / 340) + 36.53;

    printf("Temperature: %f\n", temperatureDegrees);

    // printf("My Y rotation complementary filter: %f\n", yRotationComplementary);
     
    xRotation = get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);
    yRotation = get_y_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);   

    printf("My X rotation: %f\n", xRotation);
    printf("My Y rotation: %f\n", yRotation);   

    dt = time(NULL) - timer;
    xRotationComplementary = kalman_get_angle(Kalman *kalmanFilter, float xRotation, float gyroX_scaled, float dt);

    printf("My X rotation complementary filter: %f\n", xRotationComplementary);

   return TRUE;
}

int main(int argc, char *argv[])
{
  gtk_init (&argc, &argv);

  builder = gtk_builder_new ();
  gtk_builder_add_from_file (builder, "Main.glade", NULL);

  window = GTK_WIDGET(gtk_builder_get_object (builder, "mainWindow"));
  buttonClose = GTK_BUTTON(gtk_builder_get_object (builder, "buttonClose"));
  levelValue = GTK_LABEL(gtk_builder_get_object (builder, "levelValue"));
  rotationValue = GTK_LABEL(gtk_builder_get_object (builder, "rotationValue"));
  rotationCircle = GTK_WIDGET(gtk_builder_get_object (builder, "draw"));

  gtk_builder_connect_signals (builder, NULL);
  gtk_window_maximize (GTK_WINDOW(window));

  fd = wiringPiI2CSetup (0x68);
  wiringPiI2CWriteReg8 (fd, 0x6B, 0x00);
  printf("set 0x6B=%X\n",wiringPiI2CReadReg8 (fd,0x6B));

  void kalman_init(Kalman *kalmanFilter);
  timer = time(NULL);

  g_timeout_add(100, (GSourceFunc) updateSensorData, NULL);
  g_timeout_add(100, (GSourceFunc) updateRotation, rotationCircle);
  g_timeout_add(500, (GSourceFunc) updateReadings, NULL);

  gtk_widget_show (window);

  gtk_main ();      
  return 0;
}
#include <gtk/gtk.h>
#include <math.h>
#include <cairo.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <stdio.h>
#include <time.h>
#include "kalman.h"
#include "six_axis_comp_filter.h"

int fd;
int temperature;
int acclX, acclY, acclZ;
int gyroX, gyroY, gyroZ;
float acclX_scaled, acclY_scaled, acclZ_scaled, xRotation, yRotation;
float gyroX_scaled, gyroY_scaled, gyroZ_scaled, temperatureDegrees;
float *xRotationComplementary, *yRotationComplementary, xRotationKalman, yRotationKalman;

GtkLabel *levelValueDegrees, *rotationValueDegrees, *levelValuePercent, *rotationValuePercent;
guint width, height;
GtkStyleContext *context;
GtkWidget *window, *rotationCircle;
GtkButton *buttonClose;
GtkBuilder *builder;
cairo_t *cr;

time_t timer;
float dt;
Kalman *kalmanFilter;
SixAxis *CompFilter;

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
    gchar levelDegrees[100], rotationDegrees[100], levelPercent[100], rotationPercent[100];

    sprintf(levelDegrees, "%.1f°", xRotation);
    gtk_label_set_text (GTK_LABEL(levelValueDegrees), levelDegrees);

    sprintf(levelPercent, "%.1f%%", round(tan((xRotation / 360) * 2 * M_PI) * 10000));
    gtk_label_set_text (GTK_LABEL(levelValuePercent), levelPercent);

    sprintf(rotationDegrees, "%.1f°", yRotation);
    gtk_label_set_text (GTK_LABEL(rotationValueDegrees), rotationDegrees);
    
    sprintf(rotationPercent, "%.1f%%", (100 * tan(yRotation)));
    gtk_label_set_text (GTK_LABEL(rotationValuePercent), rotationPercent);

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

    CompAccelUpdate(CompFilter, acclX_scaled, acclY_scaled, acclZ_scaled);
    CompGyroUpdate(CompFilter, gyroX_scaled, gyroY_scaled, gyroZ_scaled);
    CompUpdate(CompFilter);
    CompStart(CompFilter);
    CompAnglesGet(CompFilter, xRotationComplementary, yRotationComplementary);

    printf("My Y rotation complementary filter: %f\n", yRotationComplementary);

    // printf("My Y rotation complementary filter: %f\n", yRotationComplementary);
     
    xRotation = get_x_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);
    yRotation = get_y_rotation(acclX_scaled, acclY_scaled, acclZ_scaled);   

    printf("My X rotation: %f\n", xRotation);
    printf("My Y rotation: %f\n", yRotation);   

    dt = time(NULL) - timer;
    xRotationKalman = kalman_get_angle(kalmanFilter, xRotation, gyroX_scaled, dt);

    printf("My X rotation kalman filter: %f\n", xRotationKalman);

   return TRUE;
}

int main(int argc, char *argv[])
{
  gtk_init (&argc, &argv);

  builder = gtk_builder_new ();
  gtk_builder_add_from_file (builder, "Main.glade", NULL);

  window = GTK_WIDGET(gtk_builder_get_object (builder, "mainWindow"));
  buttonClose = GTK_BUTTON(gtk_builder_get_object (builder, "buttonClose"));
  levelValueDegrees = GTK_LABEL(gtk_builder_get_object (builder, "levelValueDegrees"));
  levelValuePercent = GTK_LABEL(gtk_builder_get_object (builder, "levelValuePercent"));
  rotationValueDegrees = GTK_LABEL(gtk_builder_get_object (builder, "rotationValueDegrees"));
  rotationValuePercent = GTK_LABEL(gtk_builder_get_object (builder, "rotationValuePercent"));
  rotationCircle = GTK_WIDGET(gtk_builder_get_object (builder, "draw"));

  gtk_builder_connect_signals (builder, NULL);
  gtk_window_maximize (GTK_WINDOW(window));

  fd = wiringPiI2CSetup (0x68);
  wiringPiI2CWriteReg8 (fd, 0x6B, 0x00);
  printf("set 0x6B=%X\n", wiringPiI2CReadReg8 (fd,0x6B));

  kalman_init(kalmanFilter);
  timer = time(NULL);

  CompInit(CompFilter, 0.1, 2);

  g_timeout_add(100, (GSourceFunc) updateSensorData, NULL);
  g_timeout_add(100, (GSourceFunc) updateRotation, rotationCircle);
  g_timeout_add(500, (GSourceFunc) updateReadings, NULL);

  gtk_widget_show (window);

  gtk_main ();      
  return 0;
}
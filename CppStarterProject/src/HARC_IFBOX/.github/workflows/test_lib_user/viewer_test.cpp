/**
 * @file main.cpp
 * @brief Test code for HARC matrix sensor interface parser library
 * @author kaminaga, cisneros
 * @date 2023/9/25
 * @copyright copyright (c) AIST 2023
 */

#include <HARC_IFBOX/HARC_IFBOX.h>
#include <iostream>
#include <ncurses.h>
#include <sys/ioctl.h>
#include <termios.h>

bool kbhit()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);

  tcsetattr(0, TCSANOW, &term);

  return byteswaiting > 0;
}

#define DISP_H_MIN 240.0 //! H value for level=0 in deg
#define DISP_H_MAX 0.0 //! H value for level=255 in deg
#define DISP_S_MIN 0.8 //! S value for level=0 in 0-1.0
#define DISP_S_MAX 1.0 //! S value for level=255 in 0-1.0
#define DISP_V_MIN 0.8 //! V value for level=0 in 0-1.0
#define DISP_V_MAX 1.0 //! V value for level=255 in 0-1.0

short default_rgb[1000][3];
short default_pair[1000][2];

void HSV2RGB(float H, float S, float V, short * R, short * G, short * B)
{
  float rf, gf, bf;

  if(S == 0)
  {
    rf = V;
    gf = V;
    bf = V;
  }
  else
  {
    int dh = (int)H / 60.0;
    float p, q, t;
    p = V * (1 - S);
    q = V * (1 - S * (H / 60.0 - dh));
    t = V * (1 - S * (1 - (H / 60.0 - dh)));

    switch(dh)
    {
      case 0:
        rf = V;
        gf = t;
        bf = p;
        break;
      case 1:
        rf = q;
        gf = V;
        bf = p;
        break;
      case 2:
        rf = p;
        gf = V;
        bf = t;
        break;
      case 3:
        rf = p;
        gf = q;
        bf = V;
        break;
      case 4:
        rf = t;
        gf = p;
        bf = V;
        break;
      case 5:
        rf = V;
        gf = p;
        bf = q;
        break;
      default:
        rf = V;
        gf = V;
        bf = V;
        break;
    }
  }
  *R = (short)(rf * 1000.0);
  *G = (short)(gf * 1000.0);
  *B = (short)(bf * 1000.0);
}

void Level2RGB(unsigned char level, short * R, short * G, short * B)
{
  float H, S, V, L;
  float H_min, H_max, S_min, S_max, V_min, V_max;
  H_min = DISP_H_MIN;
  H_max = DISP_H_MAX;
  S_min = DISP_S_MIN;
  S_max = DISP_S_MAX;
  V_min = DISP_V_MIN;
  V_max = DISP_V_MAX;

  L = (float)level / 255.0;
  H = (1 - L) * H_min + L * H_max;
  S = (1 - L) * S_min + L * S_max;
  V = (1 - L) * V_min + L * V_max;
  HSV2RGB(H, S, V, R, G, B);
}

WINDOW * frame_disp;
WINDOW * data_disp;
WINDOW * data_area;
WINDOW * row_label;
WINDOW * col_label;

void PresetNCursesColor(void)
{
  short R, G, B;

  for(int i = 0; i < COLORS; i++)
  {
    color_content(i, &default_rgb[i][0], &default_rgb[i][1], &default_rgb[i][2]);
    pair_content(i, &default_pair[i][0], &default_pair[i][1]);
  }

  init_color(0, 0, 0, 0);
  init_color(255, 1000, 1000, 1000);
  init_pair(0, 0, 255);
  init_pair(255, 255, 0);

  for(int i = 1; i < COLORS - 1; i++)
  {
    Level2RGB(i - 1, &R, &G, &B);
    init_color(i, R, G, B);
    init_pair(i, 0, i);
  }
}

void RestoreDefaultColor(void)
{
  for(int i = 0; i < COLORS; i++)
  {
    init_color(i, default_rgb[i][0], default_rgb[i][1], default_rgb[i][2]);
    init_pair(i, default_pair[i][0], default_pair[i][1]);
  }
}

int InitViewer(void)
{
  frame_disp = NULL;
  data_area = NULL;
  row_label = NULL;
  col_label = NULL;

  initscr();
  if(!has_colors())
  {
    printf("ncurses: no color\n");
    return -1;
  }
  if(start_color() < 0)
  {
    printf("ncurses: failed to start color\n");
    return -1;
  }

  PresetNCursesColor();

  if((frame_disp = newwin(1, 80, 0, 0)) == NULL) return -1;

  if((data_area = newwin(harc_ifbox::MAX_ROWS, harc_ifbox::MAX_COLS * 3, 2, 3)) == NULL) return -1;

  if((row_label = newwin(harc_ifbox::MAX_ROWS, 3, 2, 0)) == NULL) return -1;

  if((col_label = newwin(1, harc_ifbox::MAX_COLS * 3, 1, 3)) == NULL) return -1;

  return 0;
}

void EndViewer(void)
{
  RestoreDefaultColor();
  endwin();
}

int DumpDataArray(size_t frame, const harc_ifbox::DataMatrix & data)
{
  if(data.size() == 0) { return -1; }
  erase();
  wattrset(frame_disp, COLOR_PAIR(255));
  mvwprintw(frame_disp, 0, 10, "Frame No: %d", frame);
  wrefresh(frame_disp);

  wattrset(row_label, COLOR_PAIR(255));
  for(size_t i = 0; i < data.rows(); i++) mvwprintw(row_label, i, 0, "%d", i);
  wrefresh(row_label);

  wattrset(col_label, COLOR_PAIR(255));
  for(size_t j = 0; j < data.cols(); j++) mvwprintw(col_label, 0, 3 * j, "%d", j);
  wrefresh(col_label);

  for(int i = 0; i < data.rows(); i++)
  {
    for(int j = 0; j < data.cols(); j++)
    {
      unsigned char data_out;
      data_out = (data(i, j) > 254) ? 254 : data(i, j);

      wattrset(data_area, COLOR_PAIR(data_out + 1));
      mvwprintw(data_area, i, j * 3, " %02x", data(i, j));
    }
  }
  wrefresh(data_area);
  return 0;
}

int main(int argc, char * argv[])
{
  const char * dev = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_56AAAD9B81683EF9-if00";

  harc_ifbox::HARC_IFBOX harc(dev, 921600);

  if(InitViewer() < 0)
  {
    std::cerr << "Init ncurses failed" << std::endl;
    exit(-1);
  }

  harc.startCapture();

  while(1)
  {
    harc.capture();
    DumpDataArray(harc.frame(), harc.getPatches()[0].data());

    if(kbhit()) break;
  }

  EndViewer();

  return 0;
}

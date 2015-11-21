#ifndef GMU_GUI_H
#define GMU_GUI_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <map>
#include "grasp_modification_utility.h"

class gmu_gui: public QWidget
{
Q_OBJECT
public:
    gmu_gui(GMU& gmu_);
    ~gmu_gui();

private Q_SLOTS:
    void on_waypoint_selection_changed();
    void on_delete_button_clicked();
    void on_add_before_button_clicked();
    void on_add_last_button_clicked();

private:
    GMU& gmu;

    QVBoxLayout main_layout;

    QHBoxLayout layout1;
    QLabel waypoint_label;
    QComboBox waypoint_selection;
    QLabel object_label;
    QLineEdit object_text;

    QHBoxLayout layout2;
    QPushButton delete_button;
    QPushButton add_before_button;
    QPushButton add_last_button;

    QGridLayout layout3;
    QLabel pos_label, or_label;
    std::map<int, QLabel*> coord_label;
    std::map<int, QLineEdit*> coord_text;

    QHBoxLayout layout4;
    QLabel synergy_label;
    QSlider synergy_slider;
};

#endif //GMU_GUI_H
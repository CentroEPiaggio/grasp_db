#include "GMU_gui.h"

gmu_gui::gmu_gui(GMU& gmu_): QWidget(), gmu(gmu_)
{
    waypoint_label.setText("wp#");
    object_label.setText("obj_id");
    layout1.addWidget(&waypoint_label);
    layout1.addWidget(&waypoint_selection);
    layout1.addWidget(&object_label);
    layout1.addWidget(&object_text);

    delete_button.setText("delete");
    add_before_button.setText("add before");
    add_last_button.setText("add last");
    layout2.addWidget(&delete_button);
    layout2.addWidget(&add_before_button);
    layout2.addWidget(&add_last_button);

    pos_label.setText("pos");
    or_label.setText("or");
    coord_label[0]=new QLabel("x");
    coord_label[1]=new QLabel("y");
    coord_label[2]=new QLabel("z");
    coord_label[3]=new QLabel("r");
    coord_label[4]=new QLabel("p");
    coord_label[5]=new QLabel("y");
    for(int i=0;i<6;i++)
    {
	coord_text[i]=new QLineEdit();
	layout3.addWidget(coord_label.at(i),i%3,1+3*(i/3));
	layout3.addWidget(coord_text.at(i),i%3,2+3*(i/3));
    }
    layout3.addWidget(&pos_label,0,0);
    layout3.addWidget(&or_label,0,3);

    synergy_label.setText("synergy joint");
    synergy_slider.setOrientation(Qt::Horizontal);
    layout4.addWidget(&synergy_label);
    layout4.addWidget(&synergy_slider);
    
    main_layout.addLayout(&layout1);
    main_layout.addLayout(&layout2);
    main_layout.addLayout(&layout3);
    main_layout.addLayout(&layout4);

    setLayout(&main_layout);

    connect(&waypoint_selection,SIGNAL(currentIndexChanged(int)),this,SLOT(on_waypoint_selection_changed()));
    connect(&delete_button,SIGNAL(clicked(bool)),this,SLOT(on_delete_button_clicked()));
    connect(&add_before_button,SIGNAL(clicked(bool)),this,SLOT(on_add_before_button_clicked()));
    connect(&add_last_button,SIGNAL(clicked(bool)),this,SLOT(on_add_last_button_clicked()));
}

void gmu_gui::on_waypoint_selection_changed()
{

}

void gmu_gui::on_delete_button_clicked()
{

}

void gmu_gui::on_add_before_button_clicked()
{

}

void gmu_gui::on_add_last_button_clicked()
{

}

gmu_gui::~gmu_gui()
{

}

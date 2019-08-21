/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/pizza_test/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pizza_test {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(system_status(QString, int)), SLOT(system_status_gui(QString, int)));
    QObject::connect(&qnode, SIGNAL(pizza_number(QString, int)), SLOT(pizza_number_gui(QString, int)));

    /*********************
    ** Auto Start
    **********************/    
    qnode.init();
    ui.button_connect->setVisible(false);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*
 * These triggers whenever the button is clicked
 */
void MainWindow::on_new_order_clicked(bool check ) {

    // Add pizza
    pizza_queue.push_back(true);
    qnode.process_control(pizza_queue);
    ui.n_pizza_queue->setText(QString::number(pizza_queue.size()));    
}

void MainWindow::on_start_clicked(bool check ) {
    qnode.process_start(pizza_queue);
    ui.start->setEnabled(false);
    ui.new_order->setEnabled(false);
}

void MainWindow::on_button_connect_clicked(bool check ) {

}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
/*
 * Update robot status
 */
void MainWindow::system_status_gui(QString robot, int status){

    if (robot == "R1")
    {
        if (status == WAITING)
            ui.R1_status->setText("WAITING");
        else if (status == TOMATO)
        {
            ui.R1_status->setText("SPREADING TOMATO");
            ui.R1_status->setStyleSheet("QLabel { color: orange; }");
        }
        else if (status == CHEESE)
        {
            ui.R1_status->setText("SCATTERING CHEESE");
            ui.R1_status->setStyleSheet("QLabel { color: blue; }");
        }
        else if (status == OVEN)
        {
            ui.R1_status->setText("MOVING TO OVEN");
            ui.R1_status->setStyleSheet("QLabel { color: red; }");
        }
        else if (status == TOOL)
        {
            ui.R1_status->setText("CHANGING TOOL");
            ui.R1_status->setStyleSheet("QLabel { color: black; }");
        }
        else if (status == FINISH_TOOL)
        {
            ui.R1_status->setText("TOOL CHANGED");
            ui.R1_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_OVEN)
        {
            ui.R1_status->setText("PIZZA IN OVEN");
            ui.R1_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_CHEESE)
        {
            ui.R1_status->setText("CHEESE SCATTERED");
            ui.R1_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_TOMATO)
        {
            ui.R1_status->setText("TOMATO SPREADED");
            ui.R1_status->setStyleSheet("QLabel { color: green; }");
        }
    }
    if (robot == "R2")
    {
        if (status == WAITING)
            ui.R2_status->setText("WAITING");
        else if (status == TOMATO)
        {
            ui.R2_status->setText("SPREADING TOMATO");
            ui.R2_status->setStyleSheet("QLabel { color: orange; }");
        }
        else if (status == CHEESE)
        {
            ui.R2_status->setText("SCATTERING CHEESE");
            ui.R2_status->setStyleSheet("QLabel { color: blue; }");
        }
        else if (status == OVEN)
        {
            ui.R2_status->setText("MOVING TO OVEN");
            ui.R2_status->setStyleSheet("QLabel { color: red; }");
        }
        else if (status == TOOL)
        {
            ui.R2_status->setText("CHANGING TOOL");
            ui.R2_status->setStyleSheet("QLabel { color: black; }");
        }
        else if (status == FINISH_TOOL)
        {
            ui.R2_status->setText("TOOL CHANGED");
            ui.R2_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_OVEN)
        {
            ui.R2_status->setText("PIZZA IN OVEN");
            ui.R2_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_CHEESE)
        {
            ui.R2_status->setText("CHEESE SCATTERED");
            ui.R2_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_TOMATO)
        {
            ui.R2_status->setText("TOMATO SPREADED");
            ui.R2_status->setStyleSheet("QLabel { color: green; }");
        }
    }
    if (robot == "R3")
    {
        if (status == WAITING)
            ui.R3_status->setText("WAITING");
        else if (status == PLACE_IN_BOX)
        {
            ui.R3_status->setText("FROM OVEN TO BOX");
            ui.R3_status->setStyleSheet("QLabel { color: red; }");
        }
        else if (status == CUT)
        {
            ui.R3_status->setText("SLICING PIZZA");
            ui.R3_status->setStyleSheet("QLabel { color: orange; }");
        }
        else if (status == PACK_PIZZA)
        {
            ui.R3_status->setText("PACKING PIZZA");
            ui.R3_status->setStyleSheet("QLabel { color: blue; }");
        }
        else if (status == TOOL)
        {
            ui.R3_status->setText("CHANGING TOOL");
            ui.R3_status->setStyleSheet("QLabel { color: black; }");
        }
        else if (status == FINISH_TOOL)
        {
            ui.R3_status->setText("TOOL CHANGED");
            ui.R3_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_PACK_PIZZA)
        {
            ui.R3_status->setText("PIZZA PACKED");
            ui.R3_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_CUT)
        {
            ui.R3_status->setText("PIZZA SLICED");
            ui.R3_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_PLACE_IN_BOX)
        {
            ui.R3_status->setText("PIZZA IN BOX");
            ui.R3_status->setStyleSheet("QLabel { color: green; }");
        }
    }
    if (robot == "R4")
    {
        if (status == WAITING)
            ui.R4_status->setText("WAITING");
        else if (status == PLACE_IN_BOX)
        {
            ui.R4_status->setText("FROM OVEN TO BOX");
            ui.R4_status->setStyleSheet("QLabel { color: red; }");
        }
        else if (status == CUT)
        {
            ui.R4_status->setText("SLICING PIZZA");
            ui.R4_status->setStyleSheet("QLabel { color: orange; }");
        }
        else if (status == PACK_PIZZA)
        {
            ui.R4_status->setText("PACKING PIZZA");
            ui.R4_status->setStyleSheet("QLabel { color: blue; }");
        }
        else if (status == TOOL)
        {
            ui.R4_status->setText("CHANGING TOOL");
            ui.R4_status->setStyleSheet("QLabel { color: black; }");
        }
        else if (status == FINISH_TOOL)
        {
            ui.R4_status->setText("TOOL CHANGED");
            ui.R4_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_PACK_PIZZA)
        {
            ui.R4_status->setText("PIZZA PACKED");
            ui.R4_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_CUT)
        {
            ui.R4_status->setText("PIZZA SLICED");
            ui.R4_status->setStyleSheet("QLabel { color: green; }");
        }
        else if (status == FINISH_PLACE_IN_BOX)
        {
            ui.R4_status->setText("PIZZA IN BOX");
            ui.R4_status->setStyleSheet("QLabel { color: green; }");
        }
    }
}

/*
 * Update pizza numbers
 */
void MainWindow::pizza_number_gui(QString phase, int number){
    if (phase == "Order")
    {
         ui.n_pizza_queue->setText(QString::number(number));
    }
    if (phase == "Oven")
    {
         ui.n_pizza_oven->setText(QString::number(number));
    }
    if (phase == "Packed")
    {
         ui.n_pizza_packed->setText(QString::number(number));
    }
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/
void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/
void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace pizza_test


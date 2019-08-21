/**
 * @file /include/pizza_test/main_window.hpp
 *
 * @brief Qt based gui for pizza_test.
 *
 * @date November 2010
 **/
#ifndef pizza_test_MAIN_WINDOW_H
#define pizza_test_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace pizza_test {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event); // Overloaded function
    std::vector<bool> pizza_queue;


    // Status
#define WAITING         0
#define TOMATO      1
#define FINISH_TOMATO      11
#define CHEESE     2
#define FINISH_CHEESE     22
#define OVEN     3
#define FINISH_OVEN     33

#define PLACE_IN_BOX      1
#define FINISH_PLACE_IN_BOX      11
#define CUT     2
#define FINISH_CUT     22
#define PACK_PIZZA     3
#define FINISH_PACK_PIZZA     33

#define TOOL     4
#define FINISH_TOOL    44

public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check );
    void on_new_order_clicked(bool check);
    void on_start_clicked(bool check );

    /******************************************
    ** Manual connections
    *******************************************/
    void system_status_gui(QString robot, int status);
    void pizza_number_gui(QString phase, int number);

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
};

}  // namespace pizza_test

#endif // pizza_test_MAIN_WINDOW_H

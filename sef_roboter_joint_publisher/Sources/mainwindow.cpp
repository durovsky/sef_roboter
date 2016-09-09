#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_button_move_to_home_clicked()
{

}

void MainWindow::on_button_move_to_goal_slider_clicked()
{

}

void MainWindow::on_button_move_to_goal_spinbox_clicked()
{

}

void MainWindow::on_slider_joint_1_valueChanged(int value)
{
    joint_1_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_2_valueChanged(int value)
{
    joint_2_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_3_valueChanged(int value)
{
    joint_3_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_4_valueChanged(int value)
{
    joint_4_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_5_valueChanged(int value)
{
    joint_5_slider_goal = double(value)/10;
}

void MainWindow::on_slider_joint_6_valueChanged(int value)
{
    joint_6_slider_goal = double(value)/10;
}

void MainWindow::on_spinbox_joint_1_valueChanged(double arg1)
{
    joint_1_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_2_valueChanged(double arg1)
{
    joint_2_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_3_valueChanged(double arg1)
{
    joint_3_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_4_valueChanged(double arg1)
{
    joint_4_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_5_valueChanged(double arg1)
{
    joint_5_spinbox_goal = arg1;
}

void MainWindow::on_spinbox_joint_6_valueChanged(double arg1)
{
    joint_6_spinbox_goal = arg1;
}

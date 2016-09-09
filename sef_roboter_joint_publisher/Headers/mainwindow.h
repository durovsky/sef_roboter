#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_button_move_to_home_clicked();
    void on_button_move_to_goal_slider_clicked();
    void on_button_move_to_goal_spinbox_clicked();
    void on_slider_joint_1_valueChanged(int value);
    void on_slider_joint_2_valueChanged(int value);
    void on_slider_joint_3_valueChanged(int value);
    void on_slider_joint_4_valueChanged(int value);
    void on_slider_joint_5_valueChanged(int value);
    void on_slider_joint_6_valueChanged(int value);
    void on_spinbox_joint_1_valueChanged(double arg1);
    void on_spinbox_joint_2_valueChanged(double arg1);
    void on_spinbox_joint_3_valueChanged(double arg1);
    void on_spinbox_joint_4_valueChanged(double arg1);
    void on_spinbox_joint_5_valueChanged(double arg1);
    void on_spinbox_joint_6_valueChanged(double arg1);

private:
    Ui::MainWindow *ui;

    double joint_1_actual_value;
    double joint_2_actual_value;
    double joint_3_actual_value;
    double joint_4_actual_value;
    double joint_5_actual_value;
    double joint_6_actual_value;

    double joint_1_slider_goal;
    double joint_2_slider_goal;
    double joint_3_slider_goal;
    double joint_4_slider_goal;
    double joint_5_slider_goal;
    double joint_6_slider_goal;

    double joint_1_spinbox_goal;
    double joint_2_spinbox_goal;
    double joint_3_spinbox_goal;
    double joint_4_spinbox_goal;
    double joint_5_spinbox_goal;
    double joint_6_spinbox_goal;


};

#endif // MAINWINDOW_H

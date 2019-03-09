/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 2/15/19.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/
#include <utility>
#include <stdio.h>
#include <unistd.h>

#include <QtGui/QIcon>
#include <QtWidgets/QMenu>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QFileDialog>

#include <muduo/base/Thread.h>
#include <muduo/base/Logging.h>
#include <muduo/net/TcpClient.h>
#include <muduo/net/EventLoop.h>

#include "OSGWidget.h"
#include "MainWindow.h"
#include "cmd.task.pb.h"


//using namespace core;
using namespace muduo;
using namespace muduo::net;

MainWindow::MainWindow(QWidget *parent)
        : QMainWindow(parent),
          open_file_action_(nullptr),
          pool(&loop, "tcp-client") {

    this->setWindowTitle("SpaceCloud");
//
//    osgwidget_ = new OSGWidget(this);
//    this->setCentralWidget(osgwidget_);
//    osgwidget_->init();

    pool.setThreadNum(5);
    pool.start();
    initUI();
}

void MainWindow::initUI() {
    createMenu();
    createToolBar();
    createDockWidget();
}

void MainWindow::open() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Model"),
                                                    "/home/zhihui/workspace/data/osg_data",
                                                    tr("Image Files (*.osg *.osgt *.osgb)"));
    if (fileName.isEmpty()) return;

    QFileInfo f(fileName);

    uint16_t tcpport = 2000;
    std::string hostIp = "127.0.0.1";

    InetAddress serverAddr(hostIp, tcpport);
    LOG_WARN << "Connecting " << serverAddr.toIpPort();

    holder.emplace_back(new EchoClient(pool.getNextLoop(), serverAddr, 256));
    holder.back()->connect();

    cmd::task task;
    task.set_ip(hostIp);
    task.set_port(tcpport);
    task.set_pid(getpid());
    task.set_id(1);
    {
        cmd::task_job *job = task.add_jobs();
        job->set_type(cmd::task_JobType_FILE_TRANSFER);
        job->set_file_path(f.filePath().toStdString());
    }
    std::string msg = task.SerializeAsString();

    sleep(3);
    holder.back()->send(msg);
}

void MainWindow::createMenu() {
    open_file_action_ = new QAction(tr("Open"), this);
    open_file_action_->setIcon(QIcon(":/images/file_open.png"));
    connect(open_file_action_, &QAction::triggered, this, &MainWindow::open);
}

void MainWindow::createToolBar() {
    QToolBar *toolBar = addToolBar("Tools");

    toolBar->addAction(open_file_action_);
    toolBar->addSeparator();
}

void MainWindow::createDockWidget() {
    tree_widget_ = new QTreeWidget(this);
    tree_widget_->setColumnCount(1);
    tree_widget_->setHeaderHidden(true);
    //tree_widget_->setColumnWidth(0, 100);
    //tree_widget_->setStyleSheet("QTreeWidget::item {height:25px;");

    {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree_widget_, QStringList("Position"));
        item->setExpanded(true);
    }

    dock_widget_ = new QDockWidget("Status", this);
    dock_widget_->setFixedWidth(200);
    dock_widget_->setFeatures(QDockWidget::AllDockWidgetFeatures);
    dock_widget_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    dock_widget_->setWidget(tree_widget_);
    this->addDockWidget(Qt::LeftDockWidgetArea, dock_widget_);

    //QTreeWidget connect
    //connect(edit_widget_, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetClicked(QTreeWidgetItem *, int)));
    //connect(edit_widget_, SIGNAL(itemDoubleClicked(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetDoubleClicked(QTreeWidgetItem *, int)));
    //connect(edit_widget_, SIGNAL(itemPressed(QTreeWidgetItem *, int)), this, SLOT(TreeWidgetRightedClicked(QTreeWidgetItem *, int)));
}

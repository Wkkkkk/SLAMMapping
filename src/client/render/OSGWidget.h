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

#ifndef SPACECLOUD_OSGWIDGET_H
#define SPACECLOUD_OSGWIDGET_H

#include <QtWidgets/QWidget>
#include <QtWidgets/QGridLayout>
#include <QtCore/QTimer>
#include <QtCore/QFileInfo>
#include <QtGui/QPaintEvent>

#include <osg/ref_ptr>
#include <osg/Vec3d>
#include <osg/Camera>
#include <osg/Switch>
#include <osg/Geode>
#include <osgViewer/View>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgGA/TrackballManipulator>
#include <osgGA/TerrainManipulator>

//forward declaration
namespace osgQt { class GraphicsWindowQt; };

/**
 * @brief the renderer
 * It renders the whole osg scene
 * and communicate with the main window(user input)
 * through signal&slot
 * */
class OSGWidget : public QWidget, public osgViewer::CompositeViewer {
Q_OBJECT
public:
    explicit OSGWidget(QWidget *parent = nullptr);

    ~OSGWidget() final = default;

    Q_DISABLE_COPY(OSGWidget);
public:
    void init();

    void readDataFromFile(const QFileInfo &file_path);

private:
    /**
     * @brief update the scene
     * */
    void paintEvent(QPaintEvent *event) final;

    /**
     * @brief init all node
     **/
    void initSceneGraph();

    /**
     * @brief init the scene
     **/
    void initCamera();

    /**
     * @brief init some help node
     **/
    void initHelperNode();

    /**
     * @brief init hud node
     **/
    osg::Camera *createHUD();

    /**
     * @brief create the graphic context
     * @param features
     **/
    osgQt::GraphicsWindowQt *createGraphicsWindow(int x, int y, int w, int h, const std::string &name = "",
                                                  bool windowDecoration = false) const;

    /**
     * @brief calculate the bounding box of a node
     * @param node
     * @retval bounding box geode
     **/
    osg::Geode *calculateBBoxForModel(osg::Node *node) const;

    //viewer
    osg::ref_ptr<osgViewer::View> main_view_;

    //root node of the scene
    osg::ref_ptr<osg::Switch> root_node_;
    osg::ref_ptr<osg::Switch> text_node_;

    //update the scene by call frame()
    QScopedPointer<QTimer> update_timer_;
public slots:
};

#endif //SPACECLOUD_OSGWIDGET_H

/****************************************************************************\
* Copyright (C) 2026 pmdtechnologies gmbh
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
* GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
\****************************************************************************/

#ifdef ROS1
#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>
#else
#include <pluginlib/class_list_macros.hpp>
#endif

#include "o3p_node/o3p_panel.hpp"

namespace o3p_node {
O3PRvizPanel::O3PRvizPanel(QWidget *parent) : Panel(parent) {
    QVBoxLayout *main_layout = new QVBoxLayout();
    QHBoxLayout *cam_layout = new QHBoxLayout();
    QHBoxLayout *ai_layout = new QHBoxLayout();
    QHBoxLayout *param_layout = new QHBoxLayout();
    QHBoxLayout *config_layout = new QHBoxLayout();

    QLabel *cameraSelectLabel = new QLabel("Camera:");
    cameraCombo = new QComboBox();
    serialLabel = new QLabel("S/N: -");

    aiCheckBox = new QCheckBox("Enable AI");
    aiStatusLabel = new QLabel("");

    QLabel *presetLabel = new QLabel("Preset:");
    presetCombo = new QComboBox();
    QLabel *fpsLabel = new QLabel("FPS:");
    fpsCombo = new QComboBox();
    QLabel *hdrLabel = new QLabel("HDR:");
    hdrCombo = new QComboBox();
    QLabel *rangeLabel = new QLabel("Range:");
    rangeCombo = new QComboBox();

    QLabel *configLabel = new QLabel("Config:");
    configModeCombo = new QComboBox();
    configModeCombo->addItem("Get");
    configModeCombo->addItem("Set");
    configKeyEdit = new QLineEdit();
    configKeyEdit->setPlaceholderText("key");
    configValueEdit = new QLineEdit();
    configValueEdit->setPlaceholderText("value");
    configValueEdit->setReadOnly(true); // "Get" is the initial mode
    configSendButton = new QPushButton("Send");

    main_layout->addLayout(cam_layout);
    main_layout->addLayout(param_layout);
    main_layout->addLayout(config_layout);
    main_layout->addLayout(ai_layout);

    cam_layout->addWidget(cameraSelectLabel);
    cam_layout->addWidget(cameraCombo);
    cam_layout->addWidget(serialLabel);
    cam_layout->addStretch();

    ai_layout->addWidget(aiCheckBox);
    ai_layout->addWidget(aiStatusLabel);

    param_layout->addWidget(presetLabel);
    param_layout->addWidget(presetCombo);
    param_layout->addWidget(fpsLabel);
    param_layout->addWidget(fpsCombo);
    param_layout->addWidget(hdrLabel);
    param_layout->addWidget(hdrCombo);
    param_layout->addWidget(rangeLabel);
    param_layout->addWidget(rangeCombo);
    param_layout->addStretch();

    config_layout->addWidget(configLabel);
    config_layout->addWidget(configModeCombo);
    config_layout->addWidget(configKeyEdit);
    config_layout->addWidget(configValueEdit);
    config_layout->addWidget(configSendButton);

    setLayout(main_layout);

    connect(aiCheckBox, &QCheckBox::toggled, this, &O3PRvizPanel::onAIToggled);
    connect(cameraCombo, &QComboBox::currentTextChanged, this, &O3PRvizPanel::onCameraSelected);

    connect(presetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &O3PRvizPanel::onPresetSelected);
    connect(fpsCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &O3PRvizPanel::onFpsSelected);
    connect(hdrCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &O3PRvizPanel::onHdrSelected);
    connect(rangeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &O3PRvizPanel::onRangeSelected);

    connect(configModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &O3PRvizPanel::onConfigModeChanged);
    connect(configSendButton, &QPushButton::clicked, this, &O3PRvizPanel::onConfigSend);
    connect(configKeyEdit, &QLineEdit::returnPressed, this, &O3PRvizPanel::onConfigSend);
    connect(configValueEdit, &QLineEdit::returnPressed, this, &O3PRvizPanel::onConfigSend);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &O3PRvizPanel::updateLabels);
    timer->start(33);
}

O3PRvizPanel::~O3PRvizPanel() {
    // Stop the timer before any members are destroyed. Otherwise it can still fire
    // updateLabels() (which spins the node and touches the subscriptions) while those
    // objects are being torn down, leading to heap corruption / double free on close.
    if (timer) {
        timer->stop();
    }
}

void O3PRvizPanel::onInitialize() {
#ifdef ROS1
    nh = ros::NodeHandle();
#else
    node = std::make_shared<rclcpp::Node>("o3p_rviz_panel");
#endif
    // Discover the cameras that are currently publishing and fill the combobox.
    // Subscriptions are created for the selected camera in subscribeToCamera().
    refreshCameraList();
}

void O3PRvizPanel::refreshCameraList() {
    std::set<std::string> cameras;
    const std::string suffix = "/frame_depth";

#ifdef ROS1
    ros::master::V_TopicInfo topicInfos;
    ros::master::getTopics(topicInfos);
    for (const auto &info : topicInfos) {
        const std::string &name = info.name;
#else
    const auto topics = node->get_topic_names_and_types();
    for (const auto &entry : topics) {
        const std::string &name = entry.first;
#endif
        if (name.size() > suffix.size() &&
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) {
            std::string cam = name.substr(0, name.size() - suffix.size());
            if (!cam.empty() && cam.front() == '/') {
                cam.erase(0, 1);
            }
            if (!cam.empty()) {
                cameras.insert(cam);
            }
        }
    }

    updateCameraCombo(cameras);
}

void O3PRvizPanel::updateCameraCombo(const std::set<std::string> &cameras) {
    QStringList desired;
    for (const auto &cam : cameras) {
        desired << QString::fromStdString(cam);
    }

    QStringList existing;
    for (int i = 0; i < cameraCombo->count(); ++i) {
        existing << cameraCombo->itemText(i);
    }

    // Nothing changed, keep the current selection and subscriptions untouched.
    if (existing == desired) {
        return;
    }

    const QString selected = cameraCombo->currentText();

    cameraCombo->blockSignals(true);
    cameraCombo->clear();
    cameraCombo->addItems(desired);
    cameraCombo->blockSignals(false);

    const int idx = cameraCombo->findText(selected);
    if (idx >= 0) {
        // Keep the previously selected camera if it is still available.
        cameraCombo->setCurrentIndex(idx);
    } else if (cameraCombo->count() > 0) {
        // Previous camera is gone (or none was selected yet); select the first one.
        cameraCombo->setCurrentIndex(0);
        onCameraSelected(cameraCombo->currentText());
    }
}

void O3PRvizPanel::onCameraSelected(const QString &cameraName) {
    if (cameraName.isEmpty() || cameraName == currentCamera) {
        return;
    }
    subscribeToCamera(cameraName);
}

void O3PRvizPanel::subscribeToCamera(const QString &cameraName) {
    if (cameraName.isEmpty()) {
        return;
    }

    const std::string prefix = "/" + cameraName.toStdString();

    // Reset the serial number; it is filled again from the selected camera's
    // latched serial_number topic.
    currentSerial.clear();

    // Reset the cached parameter state so the comboboxes are rebuilt from the newly
    // selected camera's latched parameters topic.
    pendingParameters.clear();
    appliedParameters.clear();
    selectedPreset.clear();

    // Reset the cached config result so the value edit is cleared until the newly
    // selected camera answers a request.
    pendingConfigResult.clear();
    appliedConfigResult.clear();
    pendingConfigKey.clear();

#ifdef ROS1
    // The serial_number topic is latched, so a value is delivered even though it is
    // only published once when the camera node starts.
    serialSub = nh.subscribe(prefix + "/serial_number", 1, &O3PRvizPanel::callbackSerial, this);

    enableAIClient = nh.serviceClient<std_srvs::SetBool>(prefix + "/enable_ai");

    // The parameters topic is latched, so the current use case state is delivered as
    // soon as the panel subscribes. Commands are sent on the set_parameters topic.
    parametersSub = nh.subscribe(prefix + "/parameters", 1, &O3PRvizPanel::callbackParameters, this);
    setParametersPub = nh.advertise<std_msgs::String>(prefix + "/set_parameters", 10);

    // Generic config (extended options) get/set. Requests go out on set_config, the
    // resulting value is delivered on the latched config_result topic.
    setConfigPub = nh.advertise<std_msgs::String>(prefix + "/set_config", 10);
    configResultSub = nh.subscribe(prefix + "/config_result", 1, &O3PRvizPanel::callbackConfigResult, this);
#else
    // The serial_number topic is published with transient_local (latched) durability,
    // so the subscription must match it to receive the single published value.
    serialSub = node->create_subscription<std_msgs::msg::String>(
        prefix + "/serial_number", rclcpp::QoS(1).transient_local(),
        std::bind(&O3PRvizPanel::callbackSerial, this, std::placeholders::_1));

    enableAIClient = node->create_client<std_srvs::srv::SetBool>(prefix + "/enable_ai");

    // The parameters topic is published with transient_local (latched) durability, so
    // the subscription must match it to receive the current use case state. Commands
    // are sent on the set_parameters topic.
    parametersSub = node->create_subscription<std_msgs::msg::String>(
        prefix + "/parameters", rclcpp::QoS(1).transient_local(),
        std::bind(&O3PRvizPanel::callbackParameters, this, std::placeholders::_1));
    setParametersPub = node->create_publisher<std_msgs::msg::String>(prefix + "/set_parameters", 10);

    // Generic config (extended options) get/set. Requests go out on set_config, the
    // resulting value is delivered on the latched (transient_local) config_result topic.
    setConfigPub = node->create_publisher<std_msgs::msg::String>(prefix + "/set_config", 10);
    configResultSub = node->create_subscription<std_msgs::msg::String>(
        prefix + "/config_result", rclcpp::QoS(1).transient_local(),
        std::bind(&O3PRvizPanel::callbackConfigResult, this, std::placeholders::_1));
#endif

    currentCamera = cameraName;
}

#ifdef ROS1
void O3PRvizPanel::callbackSerial(const std_msgs::String::ConstPtr &msg)
#else
void O3PRvizPanel::callbackSerial(const std_msgs::msg::String::SharedPtr msg)
#endif
{
    if (!msg) {
        return;
    }
    currentSerial = QString::fromStdString(msg->data);
}

#ifdef ROS1
void O3PRvizPanel::callbackParameters(const std_msgs::String::ConstPtr &msg)
#else
void O3PRvizPanel::callbackParameters(const std_msgs::msg::String::SharedPtr msg)
#endif
{
    if (!msg) {
        return;
    }
    // Store the description; the comboboxes are (re)built from it in updateLabels()
    // which runs on the Qt timer, only when the description actually changed.
    pendingParameters = QString::fromStdString(msg->data);
}

#ifdef ROS1
void O3PRvizPanel::callbackConfigResult(const std_msgs::String::ConstPtr &msg)
#else
void O3PRvizPanel::callbackConfigResult(const std_msgs::msg::String::SharedPtr msg)
#endif
{
    if (!msg) {
        return;
    }
    // Store the raw "key=value" result; the value edit is updated in updateLabels()
    // (Qt timer thread) so the widget is not touched from the ROS callback.
    pendingConfigResult = QString::fromStdString(msg->data);
}

void O3PRvizPanel::updateLabels() {
#ifndef ROS1
    // Do not touch the node once the ROS context is shutting down; spinning during
    // teardown can corrupt the heap.
    if (!node || !rclcpp::ok()) {
        return;
    }
    rclcpp::spin_some(node);
#endif

    // Periodically re-scan the graph so cameras that appear or disappear are
    // reflected in the combobox (~1 s at the 33 ms timer interval).
    if (++refreshCounter >= 30) {
        refreshCounter = 0;
        refreshCameraList();
    }

    // Rebuild the parameter comboboxes only when the camera node reported a new state.
    if (pendingParameters != appliedParameters) {
        rebuildParameterCombos();
    }

    // Show a new config result in the value edit when one arrives.
    if (pendingConfigResult != appliedConfigResult) {
        appliedConfigResult = pendingConfigResult;
        const int eq = pendingConfigResult.indexOf('=');
        const QString resultKey = eq >= 0 ? pendingConfigResult.left(eq) : QString();
        const QString resultValue = eq >= 0 ? pendingConfigResult.mid(eq + 1) : pendingConfigResult;
        // Only display results for the key the user last requested; ignore stale
        // latched values from a previously selected camera.
        if (pendingConfigKey.isEmpty() || resultKey == pendingConfigKey) {
            configValueEdit->setText(resultValue);
        }
    }

    serialLabel->setText(QString("S/N: %1").arg(currentSerial.isEmpty() ? QString("-") : currentSerial));
}

void O3PRvizPanel::onAIToggled(bool checked) {
#ifdef ROS1
    std_srvs::SetBool srv;
    srv.request.data = checked;
    if (enableAIClient.call(srv)) {
        aiStatusLabel->setText(QString::fromStdString(srv.response.message));
    } else {
        aiStatusLabel->setText("Service call failed");
        aiCheckBox->blockSignals(true);
        aiCheckBox->setChecked(!checked);
        aiCheckBox->blockSignals(false);
    }
#else
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = checked;

    if (!enableAIClient->wait_for_service(std::chrono::seconds(1))) {
        aiStatusLabel->setText("Service not available");
        aiCheckBox->blockSignals(true);
        aiCheckBox->setChecked(!checked);
        aiCheckBox->blockSignals(false);
        return;
    }

    auto future = enableAIClient->async_send_request(request,
                                                     [this, checked](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture result) {
                                                         auto response = result.get();
                                                         aiStatusLabel->setText(QString::fromStdString(response->message));
                                                         if (!response->success) {
                                                             aiCheckBox->blockSignals(true);
                                                             aiCheckBox->setChecked(!checked);
                                                             aiCheckBox->blockSignals(false);
                                                         }
                                                     });
#endif
}

void O3PRvizPanel::sendParameter(const QString &key, const QString &value) {
#ifdef ROS1
    std_msgs::String msg;
    msg.data = (key + "=" + value).toStdString();
    setParametersPub.publish(msg);
#else
    if (!setParametersPub) {
        return;
    }
    std_msgs::msg::String msg;
    msg.data = (key + "=" + value).toStdString();
    setParametersPub->publish(msg);
#endif
}

void O3PRvizPanel::rebuildParameterCombos() {
    appliedParameters = pendingParameters;

    // Parse the line-based "key=value" description published by the camera node.
    QString presetsField;
    QString fpsField;
    QString fpsAllField;
    QString hdrField;
    QString rangeField;
    QString rangeAllField;

    const QStringList lines = pendingParameters.split('\n');
    for (const QString &line : lines) {
        const int eq = line.indexOf('=');
        if (eq < 0) {
            continue;
        }
        const QString key = line.left(eq);
        const QString value = line.mid(eq + 1);
        if (key == "presets") {
            presetsField = value;
        } else if (key == "fps") {
            fpsField = value;
        } else if (key == "fps_all") {
            fpsAllField = value;
        } else if (key == "hdr") {
            hdrField = value;
        } else if (key == "range") {
            rangeField = value;
        } else if (key == "range_all") {
            rangeAllField = value;
        }
    }

    auto rangeDisplayName = [](const QString &r) -> QString {
        if (r == "short")
            return "Short Range (2m)";
        if (r == "mid")
            return "Mid Range (3m)";
        if (r == "long")
            return "Long Range (5m)";
        if (r == "extended")
            return "Extended Range (7m)";
        return r + " range";
    };

    // Preset combo: "Custom settings" plus the available presets. The current preset
    // is not reported by the device, so the user's last choice is restored.
    presetCombo->blockSignals(true);
    presetCombo->clear();
    presetCombo->addItem("Custom settings");
    if (!presetsField.isEmpty()) {
        for (const QString &name : presetsField.split('|')) {
            presetCombo->addItem(name);
        }
    }
    int presetIndex = selectedPreset.isEmpty() ? 0 : presetCombo->findText(selectedPreset);
    presetCombo->setCurrentIndex(presetIndex >= 0 ? presetIndex : 0);
    presetCombo->blockSignals(false);

    // FPS combo.
    fpsCombo->blockSignals(true);
    fpsCombo->clear();
    if (!fpsAllField.isEmpty()) {
        for (const QString &fps : fpsAllField.split('|')) {
            fpsCombo->addItem(fps + " fps", QVariant(fps));
        }
    }
    const int fpsIndex = fpsCombo->findData(QVariant(fpsField));
    if (fpsIndex >= 0) {
        fpsCombo->setCurrentIndex(fpsIndex);
    }
    fpsCombo->blockSignals(false);

    // HDR combo.
    hdrCombo->blockSignals(true);
    hdrCombo->clear();
    hdrCombo->addItem("HDR off", QVariant(QString("0")));
    hdrCombo->addItem("HDR on", QVariant(QString("1")));
    hdrCombo->setCurrentIndex(hdrField == "1" ? 1 : 0);
    hdrCombo->blockSignals(false);

    // Range combo.
    rangeCombo->blockSignals(true);
    rangeCombo->clear();
    if (!rangeAllField.isEmpty()) {
        for (const QString &range : rangeAllField.split('|')) {
            rangeCombo->addItem(rangeDisplayName(range), QVariant(range));
        }
    }
    const int rangeIndex = rangeCombo->findData(QVariant(rangeField));
    if (rangeIndex >= 0) {
        rangeCombo->setCurrentIndex(rangeIndex);
    }
    rangeCombo->blockSignals(false);
}

void O3PRvizPanel::onPresetSelected(int index) {
    if (index <= 0) {
        // "Custom settings" selected; nothing to apply.
        selectedPreset.clear();
        return;
    }
    selectedPreset = presetCombo->itemText(index);
    sendParameter("preset", selectedPreset);
}

void O3PRvizPanel::onFpsSelected(int index) {
    if (index < 0) {
        return;
    }
    // Changing an individual parameter means we are no longer on a named preset.
    selectedPreset.clear();
    sendParameter("fps", fpsCombo->itemData(index).toString());
}

void O3PRvizPanel::onHdrSelected(int index) {
    if (index < 0) {
        return;
    }
    selectedPreset.clear();
    sendParameter("hdr", hdrCombo->itemData(index).toString());
}

void O3PRvizPanel::onRangeSelected(int index) {
    if (index < 0) {
        return;
    }
    selectedPreset.clear();
    sendParameter("range", rangeCombo->itemData(index).toString());
}

void O3PRvizPanel::onConfigModeChanged(int index) {
    // index 0 == "Get": the value edit only shows the retrieved value (read-only).
    // index 1 == "Set": the value edit is used to enter the value to write.
    const bool isSet = (index == 1);
    configValueEdit->setReadOnly(!isSet);
    configValueEdit->clear();
}

void O3PRvizPanel::onConfigSend() {
    const QString key = configKeyEdit->text().trimmed();
    if (key.isEmpty()) {
        return;
    }

    QString command;
    if (configModeCombo->currentIndex() == 1) {
        // Set: "set=<key>=<value>".
        command = "set=" + key + "=" + configValueEdit->text();
    } else {
        // Get: "get=<key>". The returned value is shown in the (read-only) value edit.
        command = "get=" + key;
    }

    pendingConfigKey = key;

#ifdef ROS1
    if (!setConfigPub) {
        return;
    }
    std_msgs::String msg;
    msg.data = command.toStdString();
    setConfigPub.publish(msg);
#else
    if (!setConfigPub) {
        return;
    }
    std_msgs::msg::String msg;
    msg.data = command.toStdString();
    setConfigPub->publish(msg);
#endif
}

} // namespace o3p_node

#ifdef ROS1
PLUGINLIB_EXPORT_CLASS(o3p_node::O3PRvizPanel, rviz::Panel)
#else
PLUGINLIB_EXPORT_CLASS(o3p_node::O3PRvizPanel, rviz_common::Panel)
#endif

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

#ifndef O3P_PANEL_HPP
#define O3P_PANEL_HPP

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <set>
#include <string>

#ifdef ROS1
#include <ros/master.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#endif

namespace o3p_node {
#ifdef ROS1
class O3PRvizPanel : public rviz::Panel
#else
class O3PRvizPanel : public rviz_common::Panel
#endif
{
    Q_OBJECT
  public:
    explicit O3PRvizPanel(QWidget *parent = 0);
    ~O3PRvizPanel() override;
    void onInitialize() override;

  private:
#ifdef ROS1
    void callbackSerial(const std_msgs::String::ConstPtr &msg);
#else
    void callbackSerial(const std_msgs::msg::String::SharedPtr msg);
#endif

    void updateLabels();
    void onAIToggled(bool checked);

    void refreshCameraList();
    void updateCameraCombo(const std::set<std::string> &cameras);
    void subscribeToCamera(const QString &cameraName);
    void onCameraSelected(const QString &cameraName);

#ifdef ROS1
    void callbackParameters(const std_msgs::String::ConstPtr &msg);
#else
    void callbackParameters(const std_msgs::msg::String::SharedPtr msg);
#endif

    void rebuildParameterCombos();
    void sendParameter(const QString &key, const QString &value);
    void onPresetSelected(int index);
    void onFpsSelected(int index);
    void onHdrSelected(int index);
    void onRangeSelected(int index);

#ifdef ROS1
    void callbackConfigResult(const std_msgs::String::ConstPtr &msg);
#else
    void callbackConfigResult(const std_msgs::msg::String::SharedPtr msg);
#endif

    void onConfigModeChanged(int index);
    void onConfigSend();

#ifdef ROS1
    ros::NodeHandle nh;
    ros::ServiceClient enableAIClient;

    ros::Subscriber serialSub;
    ros::Subscriber parametersSub;
    ros::Publisher setParametersPub;
    ros::Publisher setConfigPub;
    ros::Subscriber configResultSub;
#else
    rclcpp::Node::SharedPtr node;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enableAIClient;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr serialSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr parametersSub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr setParametersPub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr setConfigPub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr configResultSub;
#endif

    QLabel *aiStatusLabel;
    QLabel *serialLabel;

    QCheckBox *aiCheckBox;

    QComboBox *cameraCombo;

    QComboBox *presetCombo;
    QComboBox *fpsCombo;
    QComboBox *hdrCombo;
    QComboBox *rangeCombo;

    // Extended options (generic config get/set). The value edit is read-only in "Get"
    // mode and editable in "Set" mode; it also shows the value returned by the camera.
    QComboBox *configModeCombo;
    QLineEdit *configKeyEdit;
    QLineEdit *configValueEdit;
    QPushButton *configSendButton;

    QString currentCamera;
    QString currentSerial;

    // Latest raw "key=value" parameter description received from the camera node and
    // the one currently reflected in the comboboxes. Combos are rebuilt only when the
    // description changes so the user's interaction is not disturbed.
    QString pendingParameters;
    QString appliedParameters;
    // Preset selected by the user; empty means "Custom settings". Preserved across
    // rebuilds so changing fps/hdr/range falls back to "Custom settings".
    QString selectedPreset;

    // Latest config result received from the camera node and the one currently shown
    // in the value edit. The value edit is updated from the Qt timer (updateLabels)
    // only when a new result arrives, so it is not touched from the ROS callback.
    QString pendingConfigResult;
    QString appliedConfigResult;
    // Key the user last sent a request for; used to ignore stale results.
    QString pendingConfigKey;

    int refreshCounter = 0;

    QTimer *timer;
};
} // namespace o3p_node

#endif // O3P_PANEL_HPP

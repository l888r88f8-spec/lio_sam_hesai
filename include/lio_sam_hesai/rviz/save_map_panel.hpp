#pragma once

#include <memory>

#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTimer>

#include <rviz_common/panel.hpp>

#include "lio_sam_hesai/rviz/save_map_controller.hpp"

namespace lio_sam_hesai
{
namespace rviz
{

/// RViz panel with a "Save Map" button for mapping sessions.
///
/// The panel only ever touches Qt widgets from the GUI thread: the
/// SaveMapController is polled by a QTimer, and the asynchronous service
/// response is stored inside the controller from the rclcpp executor thread.
/// When /lio_sam/save_map is not offered (localization mode, or mapping not
/// running) the panel disables the button and reports the service as
/// unavailable instead of failing.
class SaveMapPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SaveMapPanel(QWidget* parent = nullptr);
  ~SaveMapPanel() override;

  /// Called by RViz once the DisplayContext (and therefore the shared ROS
  /// node) is available.
  void onInitialize() override;

private Q_SLOTS:
  void OnSaveClicked();
  void PollServiceState();

private:
  bool ConfirmSave();

  QPushButton* save_button_{nullptr};
  QLabel* status_label_{nullptr};
  QTimer* poll_timer_{nullptr};
  std::unique_ptr<SaveMapController> controller_;
  QString pending_status_;
  bool pending_status_is_error_{false};
};

}  // namespace rviz
}  // namespace lio_sam_hesai

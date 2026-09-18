#include "lio_sam_hesai/rviz/save_map_panel.hpp"

#include <memory>
#include <string>
#include <utility>

#include <QDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "lio_sam_hesai/rviz/save_map_dialog.hpp"

namespace lio_sam_hesai
{
namespace rviz
{

namespace
{

constexpr int kServicePollIntervalMs = 250;
constexpr char kErrorStyle[] = "color: #b00020;";
constexpr char kSuccessStyle[] = "color: #1b5e20;";

}  // namespace

SaveMapPanel::SaveMapPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  save_button_ = new QPushButton(tr("Save Map"), this);
  save_button_->setToolTip(
    tr("Choose a destination directory and the map artifacts to export"));
  save_button_->setEnabled(false);

  status_label_ = new QLabel(tr("Waiting for the RViz ROS node ..."), this);
  status_label_->setWordWrap(true);

  auto* button_row = new QHBoxLayout();
  button_row->addWidget(save_button_);
  button_row->addStretch(1);

  auto* layout = new QVBoxLayout(this);
  layout->addLayout(button_row);
  layout->addWidget(status_label_);

  poll_timer_ = new QTimer(this);
  poll_timer_->setInterval(kServicePollIntervalMs);
  connect(poll_timer_, &QTimer::timeout, this, &SaveMapPanel::PollServiceState);
  connect(
    save_button_, &QPushButton::clicked, this, &SaveMapPanel::OnSaveClicked);
}

SaveMapPanel::~SaveMapPanel() = default;

void SaveMapPanel::onInitialize()
{
  auto* context = getDisplayContext();
  if (context == nullptr)
  {
    return;
  }
  auto node_abstraction = context->getRosNodeAbstraction().lock();
  if (!node_abstraction)
  {
    return;
  }
  rclcpp::Node::SharedPtr node = node_abstraction->get_raw_node();
  if (!node)
  {
    return;
  }

  controller_ = std::make_unique<SaveMapController>(std::move(node));
  poll_timer_->start();
  PollServiceState();
}

void SaveMapPanel::OnSaveClicked()
{
  if (!controller_ ||
    !controller_->ServiceAvailable() ||
    controller_->SaveInProgress())
  {
    PollServiceState();
    return;
  }

  SaveMapDialog dialog(this);
  if (dialog.exec() != QDialog::Accepted)
  {
    pending_status_ = tr("Map save cancelled.");
    pending_status_is_error_ = false;
    PollServiceState();
    return;
  }

  const QString destination = dialog.Destination();
  if (!ConfirmSave(destination))
  {
    pending_status_ = tr("Map save cancelled.");
    pending_status_is_error_ = false;
    PollServiceState();
    return;
  }

  // Negative resolutions use the mapping node's active map_save parameters,
  // including values from a custom launch params_file.
  if (!controller_->RequestSave(
        destination.toStdString(), dialog.SaveGroundMap(),
        dialog.SaveTiledMap()))
  {
    pending_status_ =
      tr("Could not start the map save: the service is unavailable or a save "
         "is already running.");
    pending_status_is_error_ = true;
    PollServiceState();
    return;
  }

  pending_status_ = tr("Saving map to %1, please wait ...").arg(destination);
  pending_status_is_error_ = false;
  status_label_->setStyleSheet(QString());
  status_label_->setText(pending_status_);
  save_button_->setEnabled(false);
}

void SaveMapPanel::PollServiceState()
{
  if (!controller_)
  {
    return;
  }

  const bool service_available = controller_->ServiceAvailable();
  const bool save_in_progress = controller_->SaveInProgress();
  if (const auto result = controller_->TakeResult())
  {
    const QString message = QString::fromStdString(result->message);
    if (result->outcome == SaveMapController::Outcome::kSucceeded)
    {
      pending_status_ =
        message.isEmpty() ? tr("Map saved.") : message;
      pending_status_is_error_ = false;
    }
    else
    {
      pending_status_ = message.isEmpty()
                          ? tr("Map save failed. Check the mapOptimization log "
                               "for details.")
                          : message;
      pending_status_is_error_ = true;
    }
  }

  if (save_in_progress)
  {
    status_label_->setText(pending_status_.isEmpty()
                             ? tr("Saving map, please wait ...")
                             : pending_status_);
    status_label_->setStyleSheet(QString());
  }
  else if (!service_available)
  {
    status_label_->setText(
      pending_status_is_error_ && !pending_status_.isEmpty()
        ? pending_status_
        : tr("Save service unavailable: start mapping mode to enable it."));
    status_label_->setStyleSheet(kErrorStyle);
  }
  else
  {
    status_label_->setText(
      pending_status_.isEmpty() ? tr("Ready to save map.") : pending_status_);
    status_label_->setStyleSheet(
      pending_status_is_error_ ? kErrorStyle : kSuccessStyle);
  }

  save_button_->setEnabled(service_available && !save_in_progress);
}

bool SaveMapPanel::ConfirmSave(const QString& destination)
{
  // The options service removes only the known map artifacts before exporting;
  // unrelated files in the destination survive.
  const auto answer = QMessageBox::warning(
    this,
    tr("Save map"),
    tr("Save the map to:\n%1\n\n"
       "Existing map artifacts in that directory - including the configured "
       "tiled map directory and GroundMap.pcd - will be overwritten. Unrelated files "
       "are preserved.\n\n"
       "Save the map now?")
      .arg(destination),
    QMessageBox::Yes | QMessageBox::No,
    QMessageBox::No);
  return answer == QMessageBox::Yes;
}

}  // namespace rviz
}  // namespace lio_sam_hesai

PLUGINLIB_EXPORT_CLASS(lio_sam_hesai::rviz::SaveMapPanel, rviz_common::Panel)

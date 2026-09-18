#pragma once

#include <QDialog>
#include <QString>

class QCheckBox;
class QDialogButtonBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace lio_sam_hesai
{
namespace rviz
{

/// Modal "Save Map" dialog: collects the destination directory and the
/// per-save ground/tiled artifact toggles sent to /lio_sam/save_map_options.
///
/// The dialog owns no ROS or filesystem state; it only reports the operator's
/// choices. The Save button stays disabled until a non-empty absolute
/// destination is entered, and accept() refuses anything else, so the panel
/// never sends a request the mapping node must reject for a missing path.
class SaveMapDialog : public QDialog
{
  Q_OBJECT

public:
  explicit SaveMapDialog(QWidget* parent = nullptr);

  /// Trimmed destination entered by the operator.
  QString Destination() const;
  bool SaveGroundMap() const;
  bool SaveTiledMap() const;

public Q_SLOTS:
  /// Opens a directory chooser rooted at the current field value. Cancelling
  /// the chooser leaves the field unchanged.
  void BrowseForDestination();

  /// Rejects the dialog while the destination is missing or relative.
  void accept() override;

private Q_SLOTS:
  void UpdateSaveEnabled();

private:
  QString TrimmedDestination() const;
  bool HasValidDestination() const;

  QLineEdit* destination_edit_{nullptr};
  QPushButton* browse_button_{nullptr};
  QCheckBox* ground_map_checkbox_{nullptr};
  QCheckBox* tiled_map_checkbox_{nullptr};
  QLabel* validation_label_{nullptr};
  QDialogButtonBox* button_box_{nullptr};
  QPushButton* save_button_{nullptr};
};

}  // namespace rviz
}  // namespace lio_sam_hesai

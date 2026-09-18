#include "lio_sam_hesai/rviz/save_map_dialog.hpp"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace lio_sam_hesai
{
namespace rviz
{

namespace
{

constexpr char kErrorStyle[] = "color: #b00020;";

}  // namespace

SaveMapDialog::SaveMapDialog(QWidget* parent)
: QDialog(parent)
{
  setWindowTitle(tr("Save Map"));

  destination_edit_ = new QLineEdit(this);
  destination_edit_->setObjectName("saveMapDestinationEdit");
  destination_edit_->setPlaceholderText(tr("Absolute destination directory"));
  destination_edit_->setToolTip(
    tr("Absolute directory the map is exported to. Prior map artifacts there "
       "are overwritten."));

  browse_button_ = new QPushButton(tr("Browse..."), this);
  browse_button_->setObjectName("saveMapBrowseButton");

  auto* destination_row = new QHBoxLayout();
  destination_row->addWidget(destination_edit_, 1);
  destination_row->addWidget(browse_button_);

  ground_map_checkbox_ =
    new QCheckBox(tr("Save ground map (GroundMap.pcd)"), this);
  ground_map_checkbox_->setObjectName("saveMapGroundCheckbox");
  ground_map_checkbox_->setChecked(true);

  tiled_map_checkbox_ = new QCheckBox(tr("Save tiled map"), this);
  tiled_map_checkbox_->setObjectName("saveMapTiledCheckbox");
  tiled_map_checkbox_->setChecked(true);

  validation_label_ = new QLabel(this);
  validation_label_->setObjectName("saveMapValidationLabel");
  validation_label_->setStyleSheet(kErrorStyle);
  validation_label_->setWordWrap(true);
  validation_label_->setVisible(false);

  button_box_ = new QDialogButtonBox(
    QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
  button_box_->setObjectName("saveMapButtonBox");
  save_button_ = button_box_->button(QDialogButtonBox::Save);

  auto* layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel(tr("Destination directory:"), this));
  layout->addLayout(destination_row);
  layout->addWidget(ground_map_checkbox_);
  layout->addWidget(tiled_map_checkbox_);
  layout->addWidget(validation_label_);
  layout->addWidget(button_box_);

  connect(
    destination_edit_, &QLineEdit::textChanged, this,
    &SaveMapDialog::UpdateSaveEnabled);
  connect(
    browse_button_, &QPushButton::clicked, this,
    &SaveMapDialog::BrowseForDestination);
  connect(button_box_, &QDialogButtonBox::accepted, this, &SaveMapDialog::accept);
  connect(button_box_, &QDialogButtonBox::rejected, this, &SaveMapDialog::reject);

  UpdateSaveEnabled();
}

QString SaveMapDialog::Destination() const
{
  return TrimmedDestination();
}

bool SaveMapDialog::SaveGroundMap() const
{
  return ground_map_checkbox_->isChecked();
}

bool SaveMapDialog::SaveTiledMap() const
{
  return tiled_map_checkbox_->isChecked();
}

void SaveMapDialog::BrowseForDestination()
{
  const QString selected = QFileDialog::getExistingDirectory(
    this, tr("Select map destination directory"), TrimmedDestination(),
    QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  if (!selected.isEmpty())
  {
    destination_edit_->setText(selected);
  }
}

void SaveMapDialog::accept()
{
  if (!HasValidDestination())
  {
    // The Save button is already disabled in this state; guard programmatic
    // accept() calls (for example the Enter key) as well.
    UpdateSaveEnabled();
    return;
  }
  QDialog::accept();
}

void SaveMapDialog::UpdateSaveEnabled()
{
  const QString destination = TrimmedDestination();
  const bool valid = HasValidDestination();
  save_button_->setEnabled(valid);
  validation_label_->setText(
    tr("Destination must be an absolute path, for example "
       "/home/robot/maps/site."));
  validation_label_->setVisible(!destination.isEmpty() && !valid);
}

QString SaveMapDialog::TrimmedDestination() const
{
  return destination_edit_->text().trimmed();
}

bool SaveMapDialog::HasValidDestination() const
{
  const QString destination = TrimmedDestination();
  return !destination.isEmpty() && QDir::isAbsolutePath(destination);
}

}  // namespace rviz
}  // namespace lio_sam_hesai

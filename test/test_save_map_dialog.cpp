// Focused Qt tests for the Save Map dialog that gathers the
// /lio_sam/save_map_options request: the destination directory plus the
// per-save ground/tiled artifact toggles.
//
// The tests drive the real widgets produced by SaveMapDialog so the defaults,
// the absolute-path gate before QDialog::Accepted, and the exact values the
// panel forwards to the controller are pinned without a running ROS graph.

#include <gtest/gtest.h>

#include <QApplication>
#include <QByteArray>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QLineEdit>
#include <QPushButton>

#include "lio_sam_hesai/rviz/save_map_dialog.hpp"

namespace
{

using lio_sam_hesai::rviz::SaveMapDialog;

// One QApplication for the whole binary; QDialog needs a QApplication, and the
// offscreen platform keeps the test runnable without a display server.
class SaveMapDialogTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));
    static char application_name[] = "test_save_map_dialog";
    static int argc = 1;
    static char* argv[] = {application_name, nullptr};
    application_ = new QApplication(argc, argv);
  }

  static void TearDownTestSuite()
  {
    delete application_;
    application_ = nullptr;
  }

  static QLineEdit* DestinationEdit(SaveMapDialog& dialog)
  {
    return dialog.findChild<QLineEdit*>("saveMapDestinationEdit");
  }

  static QCheckBox* GroundCheckbox(SaveMapDialog& dialog)
  {
    return dialog.findChild<QCheckBox*>("saveMapGroundCheckbox");
  }

  static QCheckBox* TiledCheckbox(SaveMapDialog& dialog)
  {
    return dialog.findChild<QCheckBox*>("saveMapTiledCheckbox");
  }

  static QDialogButtonBox* ButtonBox(SaveMapDialog& dialog)
  {
    return dialog.findChild<QDialogButtonBox*>("saveMapButtonBox");
  }

  static QPushButton* SaveButton(SaveMapDialog& dialog)
  {
    QDialogButtonBox* box = ButtonBox(dialog);
    return box == nullptr ? nullptr : box->button(QDialogButtonBox::Save);
  }

  static QPushButton* CancelButton(SaveMapDialog& dialog)
  {
    QDialogButtonBox* box = ButtonBox(dialog);
    return box == nullptr ? nullptr : box->button(QDialogButtonBox::Cancel);
  }

  static QApplication* application_;
};

QApplication* SaveMapDialogTest::application_ = nullptr;

TEST_F(SaveMapDialogTest, DefaultsToBothArtifactsAndNoDestination)
{
  SaveMapDialog dialog;
  ASSERT_NE(DestinationEdit(dialog), nullptr);
  ASSERT_NE(GroundCheckbox(dialog), nullptr);
  ASSERT_NE(TiledCheckbox(dialog), nullptr);
  ASSERT_NE(SaveButton(dialog), nullptr);
  ASSERT_NE(CancelButton(dialog), nullptr);
  EXPECT_NE(dialog.findChild<QPushButton*>("saveMapBrowseButton"), nullptr);

  EXPECT_TRUE(dialog.Destination().isEmpty());
  EXPECT_TRUE(GroundCheckbox(dialog)->isChecked());
  EXPECT_TRUE(TiledCheckbox(dialog)->isChecked());
  EXPECT_TRUE(dialog.SaveGroundMap());
  EXPECT_TRUE(dialog.SaveTiledMap());
  EXPECT_FALSE(SaveButton(dialog)->isEnabled())
    << "Save must stay disabled until a destination is entered";
}

TEST_F(SaveMapDialogTest, RequiresNonEmptyAbsoluteDestinationBeforeAccepting)
{
  SaveMapDialog dialog;

  DestinationEdit(dialog)->setText("relative/maps");
  EXPECT_FALSE(SaveButton(dialog)->isEnabled());
  dialog.accept();
  EXPECT_NE(dialog.result(), QDialog::Accepted)
    << "a relative destination must not be accepted";

  DestinationEdit(dialog)->setText("   ");
  EXPECT_FALSE(SaveButton(dialog)->isEnabled());
  dialog.accept();
  EXPECT_NE(dialog.result(), QDialog::Accepted)
    << "a whitespace-only destination must not be accepted";

  DestinationEdit(dialog)->setText("/tmp/lio_sam_dialog_maps");
  EXPECT_TRUE(SaveButton(dialog)->isEnabled());
  SaveButton(dialog)->click();
  EXPECT_EQ(dialog.result(), QDialog::Accepted);
}

TEST_F(SaveMapDialogTest, ReturnsTrimmedDestinationAndArtifactToggles)
{
  SaveMapDialog dialog;
  DestinationEdit(dialog)->setText("  /tmp/lio_sam_dialog_site  ");
  GroundCheckbox(dialog)->setChecked(true);
  TiledCheckbox(dialog)->setChecked(false);

  SaveButton(dialog)->click();

  EXPECT_EQ(dialog.result(), QDialog::Accepted);
  EXPECT_EQ(dialog.Destination(), QString("/tmp/lio_sam_dialog_site"));
  EXPECT_TRUE(dialog.SaveGroundMap());
  EXPECT_FALSE(dialog.SaveTiledMap());
}

TEST_F(SaveMapDialogTest, CancelRejectsWithoutChangingSelection)
{
  SaveMapDialog dialog;
  DestinationEdit(dialog)->setText("/tmp/lio_sam_dialog_cancel");
  CancelButton(dialog)->click();

  EXPECT_EQ(dialog.result(), QDialog::Rejected);
  // The dialog still reports the current field/checkbox state; the panel only
  // acts on it when exec() returns QDialog::Accepted.
  EXPECT_EQ(dialog.Destination(), QString("/tmp/lio_sam_dialog_cancel"));
  EXPECT_TRUE(dialog.SaveGroundMap());
  EXPECT_TRUE(dialog.SaveTiledMap());
}

}  // namespace

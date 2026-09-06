#include "geo_tuner/rviz/geo_field_panel.hpp"

#include "geo_tuner/rviz/controller_health_panel.hpp"
#include "geo_tuner/rviz/gain_panel.hpp"
#include "geo_tuner/rviz/trajectory_test_panel.hpp"
#include "geo_tuner/rviz/tuner_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <QScrollArea>
#include <QVBoxLayout>

namespace geo_tuner::panels
{

GeoFieldPanel::GeoFieldPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  tabs_ = new QTabWidget(this);
  tabs_->setDocumentMode(true);
  layout->addWidget(tabs_);

  auto * tuner = new TunerPanel(this);
  auto * gains = new GainPanel(this);
  auto * health = new ControllerHealthPanel(this);
  auto * fly = new TrajectoryTestPanel(this);

  children_ = {
    {"Tuner", tuner},
    {"Gains", gains},
    {"Health", health},
    {"Fly", fly},
  };

  // Tuner and Gains share a tab. They are one job seen twice: the conductor
  // searches for gains, the gain table is where you read the result, nudge
  // it by hand, and press Save -- a tuning session that is never written to
  // the vehicle was a session for nothing. Splitting them made the operator
  // hunt for the Save button in another tab.
  auto * tune = new QWidget(this);
  auto * tune_layout = new QVBoxLayout(tune);
  tune_layout->setContentsMargins(0, 0, 0, 0);
  tune_layout->setSpacing(2);
  tune_layout->addWidget(tuner);
  tune_layout->addWidget(gains);
  tune_layout->addStretch(1);

  // One namespace box for the shared tab: the tuner's selector drives the
  // gain panel too, so the operator never sees two boxes asking the same
  // question with room to disagree.
  gains->setEmbedded(true);
  tuner_ = tuner;
  gains_ = gains;
  connect(tuner, &TunerPanel::namespaceApplied, gains, &GainPanel::adoptNamespace);

  // The combined tab is taller than a short dock, and a control you cannot
  // scroll to is a control you do not have.
  auto * scroll = new QScrollArea(this);
  scroll->setWidget(tune);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);

  tabs_->addTab(scroll, "Tune");
  tabs_->addTab(health, "Health");
  tabs_->addTab(fly, "Fly");
}

GeoFieldPanel::~GeoFieldPanel() = default;

void GeoFieldPanel::onInitialize()
{
  // The children are ordinary Panels that RViz did not create, so nothing
  // has given them a DisplayContext -- without this they never get a ROS
  // node and sit blank forever.
  for(const auto & child : children_)
  {
    child.second->initialize(getDisplayContext());
    child.second->onInitialize();
  }
}

void GeoFieldPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  for(const auto & child : children_)
    child.second->load(config.mapGetChild(child.first));

  int index = 0;
  if(config.mapGetInt("Tab", &index) && index >= 0 && index < tabs_->count())
    tabs_->setCurrentIndex(index);

  // A config saved before the namespace rows were unified may carry two
  // different namespaces; the tuner's wins, visibly.
  if(tuner_ && gains_)
    gains_->adoptNamespace(tuner_->currentNamespace());
}

void GeoFieldPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  for(const auto & child : children_)
    child.second->save(config.mapMakeChild(child.first));
  config.mapSetValue("Tab", tabs_->currentIndex());
}

}  // namespace geo_tuner::panels

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(geo_tuner::panels::GeoFieldPanel, rviz_common::Panel)

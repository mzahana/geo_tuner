// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// One RViz panel holding all four field panels as tabs.
//
// RViz docks every panel into the same column, so four separate panels turn
// the window into a scroll and squeeze the 3D view. As tabs they occupy one
// dock, and only the tab you are using takes space. Tuner is first: it is the
// reason the set exists.
//
// The individual panels remain registered, so anyone who wants them docked
// separately still can.

#ifndef GEO_TUNER__RVIZ__GEO_FIELD_PANEL_HPP_
#define GEO_TUNER__RVIZ__GEO_FIELD_PANEL_HPP_

#include <rviz_common/panel.hpp>

#include <QTabWidget>

#include <vector>

namespace geo_tuner::panels
{

class GeoFieldPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GeoFieldPanel(QWidget * parent = nullptr);
  ~GeoFieldPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QTabWidget * tabs_{nullptr};
  std::vector<std::pair<QString, rviz_common::Panel *>> children_;
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__GEO_FIELD_PANEL_HPP_

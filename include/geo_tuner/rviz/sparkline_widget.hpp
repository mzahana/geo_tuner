// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// Minimal scrolling trace widget: a fixed-capacity ring buffer drawn as a
// polyline. Deliberately hand-drawn rather than built on a charting library
// so the panels carry no dependency beyond Qt Widgets -- these run on a
// field laptop where a missing optional package means no telemetry at all.

#ifndef GEO_TUNER__RVIZ__SPARKLINE_WIDGET_HPP_
#define GEO_TUNER__RVIZ__SPARKLINE_WIDGET_HPP_

#include <QColor>
#include <QString>
#include <QWidget>

#include <deque>

namespace geo_tuner::panels
{

class SparklineWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SparklineWidget(QWidget * parent = nullptr);

  /// Label drawn at the top-left, e.g. "pos err [m]".
  void setLabel(const QString & label);

  /// Trace colour.
  void setColor(const QColor & color);

  /// Draw a dashed horizontal line at this value (e.g. a tolerance).
  /// Pass a non-finite value to remove it.
  void setWarnLevel(double level);

  /// Lower bound on the vertical span, so a quiet signal does not get
  /// amplified into meaningless noise by autoscaling.
  void setMinSpan(double span);

  /// Pin the vertical range (e.g. 0..1 for a throttle). Pass a
  /// non-finite pair to return to autoscaling.
  void setFixedRange(double lo, double hi);

  /// Number of samples retained.
  void setCapacity(int capacity);

  void push(double value);
  void clear();

  QSize sizeHint() const override;

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  std::deque<double> samples_;
  int capacity_{240};          // 48 s at 5 Hz
  QString label_;
  QColor color_{QColor(70, 160, 230)};
  double warn_level_;
  double min_span_{0.1};
  double fixed_lo_, fixed_hi_;
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__SPARKLINE_WIDGET_HPP_

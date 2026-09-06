#include "geo_tuner/rviz/sparkline_widget.hpp"

#include <QPainter>
#include <QPaintEvent>
#include <QPen>
#include <QPolygonF>

#include <algorithm>
#include <cmath>
#include <limits>

namespace geo_tuner::panels
{

namespace
{
const double kNaN = std::numeric_limits<double>::quiet_NaN();
}

SparklineWidget::SparklineWidget(QWidget * parent)
: QWidget(parent), warn_level_(kNaN), fixed_lo_(kNaN), fixed_hi_(kNaN)
{
  setMinimumHeight(46);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void SparklineWidget::setLabel(const QString & label)
{
  label_ = label;
  update();
}

void SparklineWidget::setColor(const QColor & color)
{
  color_ = color;
  update();
}

void SparklineWidget::setWarnLevel(double level)
{
  warn_level_ = level;
  update();
}

void SparklineWidget::setMinSpan(double span)
{
  min_span_ = std::max(1e-6, span);
  update();
}

void SparklineWidget::setFixedRange(double lo, double hi)
{
  fixed_lo_ = lo;
  fixed_hi_ = hi;
  update();
}

void SparklineWidget::setCapacity(int capacity)
{
  capacity_ = std::max(2, capacity);
  while(static_cast<int>(samples_.size()) > capacity_)
    samples_.pop_front();
  update();
}

void SparklineWidget::push(double value)
{
  // Non-finite samples are dropped rather than plotted: a NaN would blow
  // up the autoscale and blank the whole trace.
  if(!std::isfinite(value))
    return;
  samples_.push_back(value);
  while(static_cast<int>(samples_.size()) > capacity_)
    samples_.pop_front();
  update();
}

void SparklineWidget::clear()
{
  samples_.clear();
  update();
}

QSize SparklineWidget::sizeHint() const
{
  return QSize(220, 46);
}

void SparklineWidget::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing, true);

  const QRectF box = rect().adjusted(0.5, 0.5, -0.5, -0.5);
  p.fillRect(box, palette().base());
  p.setPen(QPen(palette().mid().color(), 1.0));
  p.drawRect(box);

  const double top = box.top() + 12.0;   // room for the label
  const double bottom = box.bottom() - 2.0;
  const double left = box.left() + 2.0;
  const double right = box.right() - 2.0;

  // Vertical range: fixed if requested, else autoscaled over the window
  // with a floor on the span and a small margin.
  double lo, hi;
  if(std::isfinite(fixed_lo_) && std::isfinite(fixed_hi_) && fixed_hi_ > fixed_lo_)
  {
    lo = fixed_lo_;
    hi = fixed_hi_;
  }
  else if(samples_.empty())
  {
    lo = -min_span_ / 2.0;
    hi = min_span_ / 2.0;
  }
  else
  {
    const auto mm = std::minmax_element(samples_.begin(), samples_.end());
    lo = *mm.first;
    hi = *mm.second;
    if(std::isfinite(warn_level_))
    {
      lo = std::min(lo, warn_level_);
      hi = std::max(hi, warn_level_);
    }
    const double span = hi - lo;
    if(span < min_span_)
    {
      const double mid = 0.5 * (lo + hi);
      lo = mid - min_span_ / 2.0;
      hi = mid + min_span_ / 2.0;
    }
    else
    {
      lo -= 0.08 * span;
      hi += 0.08 * span;
    }
  }

  auto y_of = [&](double v) {
    const double t = (v - lo) / (hi - lo);
    return bottom - t * (bottom - top);
  };

  // Zero line, when it falls inside the visible range
  if(lo < 0.0 && hi > 0.0)
  {
    p.setPen(QPen(palette().mid().color(), 1.0, Qt::DotLine));
    p.drawLine(QPointF(left, y_of(0.0)), QPointF(right, y_of(0.0)));
  }

  if(std::isfinite(warn_level_) && warn_level_ >= lo && warn_level_ <= hi)
  {
    p.setPen(QPen(QColor(200, 120, 40), 1.0, Qt::DashLine));
    p.drawLine(QPointF(left, y_of(warn_level_)), QPointF(right, y_of(warn_level_)));
  }

  if(samples_.size() >= 2)
  {
    // The trace is right-anchored: the newest sample sits at the right
    // edge, so a partially filled buffer grows leftwards instead of
    // stretching a short history across the full width.
    const double dx = (right - left) / static_cast<double>(capacity_ - 1);
    QPolygonF poly;
    poly.reserve(static_cast<int>(samples_.size()));
    const int n = static_cast<int>(samples_.size());
    for(int i = 0; i < n; ++i)
    {
      const double x = right - (n - 1 - i) * dx;
      poly << QPointF(x, y_of(samples_[static_cast<size_t>(i)]));
    }
    p.setPen(QPen(color_, 1.6));
    p.drawPolyline(poly);
  }

  p.setPen(palette().windowText().color());
  QFont f = p.font();
  f.setPointSizeF(std::max(7.0, f.pointSizeF() - 1.0));
  p.setFont(f);
  QString head = label_;
  if(!samples_.empty())
    head += QString("  %1").arg(samples_.back(), 0, 'f', 3);
  p.drawText(QPointF(left + 3.0, box.top() + 11.0), head);

  // Range readout on the right of the label row
  const QString range = QString("%1 / %2").arg(lo, 0, 'f', 2).arg(hi, 0, 'f', 2);
  p.setPen(palette().mid().color());
  p.drawText(QRectF(left, box.top() + 1.0, right - left - 3.0, 12.0),
             Qt::AlignRight | Qt::AlignVCenter, range);
}

}  // namespace geo_tuner::panels

// Copyright (c) 2026 Mohamed Abdelkader
// SPDX-License-Identifier: MIT
//
// Shared namespace row for the field panels: a text box plus Apply, and the
// prefix rule every panel follows. Factored out so the panels cannot drift
// apart on the one thing that decides whether they see any data at all.

#ifndef GEO_TUNER__RVIZ__NAMESPACE_SELECTOR_HPP_
#define GEO_TUNER__RVIZ__NAMESPACE_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <QLineEdit>
#include <QPushButton>
#include <QWidget>

namespace geo_tuner::panels
{

class NamespaceSelector : public QWidget
{
  Q_OBJECT

public:
  explicit NamespaceSelector(QWidget * parent = nullptr);

  QString ns() const;
  void setNs(const QString & ns);

  /// Topic/service prefix, always with a leading and trailing slash.
  ///
  /// Empty box  -> the namespace RViz itself was launched in, so
  ///               `field_monitor.launch.py ns:=interceptor` needs no typing.
  /// A lone "/" -> the ROOT namespace, explicitly. Without this there is no
  ///               way to watch a stack at the root from an RViz that was
  ///               itself launched under a namespace.
  /// Anything else -> that namespace.
  QString prefix(const rclcpp::Node::SharedPtr & node) const;

Q_SIGNALS:
  /// Emitted when the user applies a namespace (button or Enter).
  void applied();

private:
  QLineEdit * edit_{nullptr};
  QPushButton * button_{nullptr};
};

}  // namespace geo_tuner::panels

#endif  // GEO_TUNER__RVIZ__NAMESPACE_SELECTOR_HPP_

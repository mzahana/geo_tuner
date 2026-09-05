#include "geo_tuner/rviz/namespace_selector.hpp"

#include <QHBoxLayout>
#include <QLabel>

namespace geo_tuner::panels
{

NamespaceSelector::NamespaceSelector(QWidget * parent)
: QWidget(parent)
{
  auto * row = new QHBoxLayout(this);
  row->setContentsMargins(0, 0, 0, 0);
  row->addWidget(new QLabel("Namespace:", this));
  edit_ = new QLineEdit(this);
  edit_->setPlaceholderText("(RViz namespace; e.g. interceptor, or / for root)");
  row->addWidget(edit_, 1);
  button_ = new QPushButton("Apply", this);
  row->addWidget(button_);

  connect(button_, &QPushButton::clicked, this, &NamespaceSelector::applied);
  connect(edit_, &QLineEdit::returnPressed, this, &NamespaceSelector::applied);
}

QString NamespaceSelector::ns() const
{
  return edit_->text();
}

void NamespaceSelector::setNs(const QString & ns)
{
  edit_->setText(ns);
}

QString NamespaceSelector::prefix(const rclcpp::Node::SharedPtr & node) const
{
  const QString raw = edit_->text().trimmed();
  if(raw == "/")
    return QString("/");

  QString ns = raw;
  if(ns.isEmpty() && node)
    ns = QString::fromStdString(node->get_effective_namespace());

  while(ns.startsWith('/'))
    ns.remove(0, 1);
  while(ns.endsWith('/'))
    ns.chop(1);
  return ns.isEmpty() ? QString("/") : QString("/%1/").arg(ns);
}

}  // namespace geo_tuner::panels

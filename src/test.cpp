#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QLabel>
#include <QWidget>
#include <QMainWindow>
class ExpandableItem : public QWidget {
    Q_OBJECT

public:
    ExpandableItem(const QString& title, QWidget* parent = nullptr)
        : QWidget(parent) {
        QVBoxLayout* layout = new QVBoxLayout(this);
        QPushButton* toggleButton = new QPushButton(title, this); // 标题按钮
        contentWidget = new QWidget(this); // 内容区域
        contentWidget->setStyleSheet("background-color: lightgray;");
        contentWidget->setVisible(false); // 默认隐藏

        QVBoxLayout* contentLayout = new QVBoxLayout(contentWidget);
        contentLayout->addWidget(new QLabel("This is content for " + title, contentWidget));

        // 将布局添加到主项
        layout->addWidget(toggleButton);
        layout->addWidget(contentWidget);

        // 连接按钮点击事件以控制展开/折叠
        connect(toggleButton, &QPushButton::clicked, this, [this]() {
            contentWidget->setVisible(!contentWidget->isVisible());
        });
    }

    // 提供接口以允许动态添加内容
    QVBoxLayout* getContentLayout() {
        return static_cast<QVBoxLayout*>(contentWidget->layout());
    }

private:
    QWidget* contentWidget; // 内容区域
};

class MainWindow : public QWidget {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr) : QWidget(parent) {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);

        // 可滚动区域
        QScrollArea* scrollArea = new QScrollArea(this);
        scrollArea->setWidgetResizable(true);

        QWidget* container = new QWidget(this);
        QVBoxLayout* containerLayout = new QVBoxLayout(container);

        // 创建多个可展开项
        ExpandableItem* item1 = new ExpandableItem("Section 1", container);
        ExpandableItem* item2 = new ExpandableItem("Section 2", container);
        ExpandableItem* nestedItem = new ExpandableItem("Nested Section", container);

        // 在嵌套项中添加更多内容
        ExpandableItem* subItem = new ExpandableItem("Sub Section", container);
        nestedItem->getContentLayout()->addWidget(subItem);

        containerLayout->addWidget(item1);
        containerLayout->addWidget(item2);
        containerLayout->addWidget(nestedItem);
        containerLayout->addStretch(); // 添加弹性空间

        scrollArea->setWidget(container);
        mainLayout->addWidget(scrollArea);
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    QMainWindow From; // 创建主窗口
    MainWindow window(&From); // 创建主窗口的内容
    
    From.show();
    

    return app.exec();
}
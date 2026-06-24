import json
import os
import matplotlib.pyplot as plt

# ================= 路径配置 =================
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 我们可以同时读取 Ours 和 VLA 的数据做对比
OURS_JSON = os.path.join(BASE_DIR, "eval_logs/tier4/ours.json")
VLA_JSON = os.path.join(BASE_DIR, "eval_logs/tier4/vla.json")

def plot_dual_failure_analysis():
    def get_fail_counts(path):
        if not os.path.exists(path): return None
        with open(path, 'r') as f:
            data = json.load(f)
        s = data['summary']
        # 对应你图中的 4 类
        return [
            s.get('perception_failure', 0),
            s.get('planning_failure', 0),
            s.get('grasp_failure', 0),
            s.get('insertion_failure', 0)
        ]

    labels = ['Perception', 'Planning', 'Grasp/Transport', 'Insertion']
    ours_counts = get_fail_counts(OURS_JSON)
    vla_counts = get_fail_counts(VLA_JSON)

    # 学术配色：深蓝、浅蓝、浅橙、深红
    colors = ['#4A90E2', '#A3CBF1', '#F5A623', '#D0021B']

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    # 绘制 Ours
    if ours_counts:
        axes[0].pie(ours_counts, labels=labels, autopct='%1.1f%%', startangle=140, 
                   colors=colors, explode=[0.05]*4, wedgeprops=dict(width=0.4))
        axes[0].set_title("Ours (Pi-0.5 Fast) Failure Modes", fontsize=14, weight='bold')
        axes[0].annotate(f'Total Fails: {sum(ours_counts)}', xy=(0,0), ha='center', weight='bold')

    # 绘制 VLA
    if vla_counts:
        axes[1].pie(vla_counts, labels=labels, autopct='%1.1f%%', startangle=140, 
                   colors=colors, explode=[0.05]*4, wedgeprops=dict(width=0.4))
        axes[1].set_title("VLA Baseline Failure Modes", fontsize=14, weight='bold')
        axes[1].annotate(f'Total Fails: {sum(vla_counts)}', xy=(0,0), ha='center', weight='bold')

    plt.tight_layout()
    plt.savefig("failure_mode_comparison.png", dpi=300)
    print("✅ 故障对比图已生成：failure_mode_comparison.png")
    plt.show()

if __name__ == "__main__":
    plot_dual_failure_analysis()
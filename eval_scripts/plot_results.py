import json
import os
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

# ================= 路径对齐 =================
# 获取当前脚本所在目录 (/home/i6user/Desktop/robot_lego/eval_scripts)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_ROOT = os.path.join(BASE_DIR, "eval_logs")

def load_data(method_name, tier_name, relative_path):
    """
    自动拼接绝对路径并加载数据
    """
    full_path = os.path.join(BASE_DIR, relative_path)
    
    if not os.path.exists(full_path):
        print(f"⚠️ 警告: 找不到文件 {full_path}，跳过该项。")
        return None

    with open(full_path, 'r') as f:
        data = json.load(f)
    
    # 提取 raw_data 并转为 DataFrame
    df = pd.DataFrame(data['raw_data'])
    df['Method'] = method_name
    df['Tier'] = tier_name
    return df

# 1. 加载所有对比数据 (使用相对于项目根目录的路径)
data_sources = [
    ("Ours (Pi-0.5 Fast)", "Tier 1", "eval_logs/tier1/ours.json"),
    ("VLA Baseline", "Tier 1", "eval_logs/tier1/vla.json"),
    ("Ours (Pi-0.5 Fast)", "Tier 4", "eval_logs/tier4/ours.json"),
    ("VLA Baseline", "Tier 4", "eval_logs/tier4/vla.json"),
]

dfs = []
for method, tier, path in data_sources:
    df = load_data(method, tier, path)
    if df is not None:
        dfs.append(df)

if not dfs:
    print("❌ 错误: 没有加载到任何有效数据，请检查路径。")
    exit()

all_df = pd.concat(dfs, ignore_index=True)

# 2. 设置学术绘图风格
plt.style.use('seaborn-v0_8-paper')
fig, axes = plt.subplots(1, 2, figsize=(15, 6))

# --- 图 A：成功率对比 (柱状图) ---
success_stats = all_df.groupby(['Method', 'Tier'])['success'].mean().reset_index()
sns.barplot(data=success_stats, x='Tier', y='success', hue='Method', ax=axes[0], palette="viridis")
axes[0].set_title("Success Rate Comparison (Assembly Success)", fontsize=14, weight='bold')
axes[0].set_ylabel("Success Rate (0.0 - 1.0)")
axes[0].set_ylim(0, 1.1)

# --- 图 B：执行精度分布 (箱线图) ---
# 过滤掉失败的任务，只统计成功任务的精度才更有意义
success_only_df = all_df[all_df['success'] == 1.0]
sns.boxplot(data=success_only_df, x='Tier', y='accuracy_error', hue='Method', ax=axes[1])
axes[1].set_yscale('log') 
axes[1].set_title("Execution Accuracy Error (Log Scale)", fontsize=14, weight='bold')
axes[1].set_ylabel("Error in Meters (Lower is Better)")

# 3. 自动调整并保存
plt.tight_layout()
output_name = "comparison_report_ours_vs_vla.png"
plt.savefig(output_name, dpi=300)
print(f"📊 完美！对比大图已生成至: {os.getcwd()}/{output_name}")
plt.show()
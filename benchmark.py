import subprocess
import re
import csv
import os
import math
import json

# Configuration
MAX_STEPS_LIST = [2, 4, 8, 12, 16, 24, 32]
TRIALS_PER_CONFIG = 15
VEHICLE_COUNT = 15
SIM_STEPS = 50

def create_headless_source():
    print("Preparing headless source code...")
    if not os.path.exists("main.c"):
        raise FileNotFoundError("Could not find main.c in the current directory.")
        
    with open("main.c", "r") as f:
        content = f.read()
    
    # 1. Bypass animation delay
    content = content.replace("usleep(300000);", "/* usleep bypassed */")
    
    # 2. Bypass terminal screen clearing
    content = content.replace('printf("\\033[2J\\033[H");', "/* clear bypassed */")
    
    # 3. Bypass rendering of the grid
    content = content.replace('draw_scene(roads, roadNum, vehicles, vehiNum, side);', "/* draw bypassed */")
    
    # 4. Bypass terminal window resizing call
    content = content.replace("resize_terminal(canvaSide + vehiNum + 10, canvaSide + 40);", "/* resize bypassed */")
    
    # 5. Bypass the final getchar() hanging check
    content = content.replace("getchar();", "/* getchar bypassed */")
    
    with open("main_headless.c", "w") as f:
        f.write(content)

def compile_headless():
    print("Compiling headless binary...")
    subprocess.run(["gcc", "-O3", "main_headless.c", "-o", "main_headless"], check=True)

def run_benchmark():
    raw_results = []
    
    for max_steps in MAX_STEPS_LIST:
        for trial in range(TRIALS_PER_CONFIG):
            print(f"Running lookahead={max_steps}, trial={trial+1}/{TRIALS_PER_CONFIG}...")
            # Run simulation with --random and --lookahead
            cmd = ["./main_headless", "--random", str(VEHICLE_COUNT), "--steps", str(SIM_STEPS), "--lookahead", str(max_steps), "--child"]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            output = proc.stdout
            
            # Parse planning times per tick
            plan_times = [float(x) for x in re.findall(r"plan (\d+\.\d+) ms", output)]
            avg_plan_time = sum(plan_times) / len(plan_times) if plan_times else 0.0
            
            # Parse number of vehicles that reached goal
            final_section = output.split("───────────────────────────────────────────────────────────")[-1]
            goals_reached = len(re.findall(r"✔ GOAL", final_section))
            success_rate = goals_reached / VEHICLE_COUNT
            
            raw_results.append({
                "maxSteps": max_steps,
                "trial": trial,
                "avg_plan_time_ms": avg_plan_time,
                "success_rate": success_rate,
                "goals_reached": goals_reached
            })
            
    # Save CSV
    csv_file = "benchmark_results.csv"
    with open(csv_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["maxSteps", "trial", "avg_plan_time_ms", "success_rate", "goals_reached"])
        writer.writeheader()
        writer.writerows(raw_results)
    print(f"CSV results saved to {csv_file}")
    return raw_results

def compute_statistics(raw_results):
    stats = {}
    for r in raw_results:
        ms = r["maxSteps"]
        if ms not in stats:
            stats[ms] = {"success_rates": [], "plan_times": []}
        stats[ms]["success_rates"].append(r["success_rate"])
        stats[ms]["plan_times"].append(r["avg_plan_time_ms"])
        
    summary = []
    for ms in MAX_STEPS_LIST:
        s_list = stats[ms]["success_rates"]
        p_list = stats[ms]["plan_times"]
        
        # Success rate stats
        mean_s = sum(s_list) / len(s_list)
        var_s = sum((x - mean_s) ** 2 for x in s_list) / (len(s_list) - 1) if len(s_list) > 1 else 0.0
        ci_s = 1.96 * math.sqrt(var_s / len(s_list)) if len(s_list) > 0 else 0.0
        
        # Plan time stats
        mean_p = sum(p_list) / len(p_list)
        var_p = sum((x - mean_p) ** 2 for x in p_list) / (len(p_list) - 1) if len(p_list) > 1 else 0.0
        ci_p = 1.96 * math.sqrt(var_p / len(p_list)) if len(p_list) > 0 else 0.0
        
        summary.append({
            "maxSteps": ms,
            "mean_success_rate": mean_s,
            "ci_success_rate": ci_s,
            "mean_plan_time_ms": mean_p,
            "ci_plan_time_ms": ci_p
        })
    return summary

def generate_html_dashboard(summary):
    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>WHCA* Performance Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {{
            font-family: 'Inter', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            background-color: #0f172a;
            color: #f8fafc;
            margin: 0;
            padding: 40px 20px;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
        }}
        header {{
            text-align: center;
            margin-bottom: 40px;
        }}
        h1 {{
            font-size: 2.5rem;
            color: #38bdf8;
            margin-bottom: 10px;
        }}
        p.subtitle {{
            color: #94a3b8;
            font-size: 1.1rem;
        }}
        .grid {{
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 30px;
            margin-bottom: 40px;
        }}
        @media (max-width: 900px) {{
            .grid {{
                grid-template-columns: 1fr;
            }}
        }}
        .card {{
            background-color: #1e293b;
            border-radius: 12px;
            padding: 24px;
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
            border: 1px solid #334155;
        }}
        h2 {{
            font-size: 1.4rem;
            margin-top: 0;
            margin-bottom: 20px;
            color: #f1f5f9;
            border-bottom: 2px solid #334155;
            padding-bottom: 10px;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin-top: 20px;
        }}
        th, td {{
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #334155;
        }}
        th {{
            color: #38bdf8;
            font-weight: 600;
        }}
        tr:hover {{
            background-color: #1e293b;
        }}
    </style>
</head>
<body>
    <div class="container">
        <header>
            <h1>WHCA* Lookahead Benchmark</h1>
            <p class="subtitle">Statistical evaluation of vehicle behavior and computational complexity under varying lookahead window sizes (maxSteps).</p>
        </header>

        <div class="grid">
            <div class="card">
                <h2>Goal Success Rate (%) vs. maxSteps</h2>
                <canvas id="successChart"></canvas>
            </div>
            <div class="card">
                <h2>Planning Time (ms/tick) vs. maxSteps</h2>
                <canvas id="timeChart"></canvas>
            </div>
        </div>

        <div class="card">
            <h2>Statistical Summary Table</h2>
            <table>
                <thead>
                    <tr>
                        <th>maxSteps</th>
                        <th>Mean Success Rate</th>
                        <th>95% Confidence Interval (Success)</th>
                        <th>Mean Planning Time (ms/tick)</th>
                        <th>95% Confidence Interval (Time)</th>
                    </tr>
                </thead>
                <tbody>
                    {"".join(f'''
                    <tr>
                        <td><strong>{item["maxSteps"]}</strong></td>
                        <td>{item["mean_success_rate"] * 100:.1f}%</td>
                        <td>&plusmn; {item["ci_success_rate"] * 100:.1f}%</td>
                        <td>{item["mean_plan_time_ms"]:.3f} ms</td>
                        <td>&plusmn; {item["ci_plan_time_ms"]:.3f} ms</td>
                    </tr>
                    ''' for item in summary)}
                </tbody>
            </table>
        </div>
    </div>

    <script>
        const summaryData = {json.dumps(summary)};
        const labels = summaryData.map(d => d.maxSteps);
        
        // Success Chart
        const successCtx = document.getElementById('successChart').getContext('2d');
        new Chart(successCtx, {{
            type: 'line',
            data: {{
                labels: labels,
                datasets: [{{
                    label: 'Mean Success Rate',
                    data: summaryData.map(d => d.mean_success_rate),
                    borderColor: '#06b6d4',
                    backgroundColor: 'rgba(6, 182, 212, 0.1)',
                    tension: 0.1,
                    fill: true,
                    borderWidth: 3,
                    pointRadius: 6,
                    pointBackgroundColor: '#06b6d4'
                }}]
            }},
            options: {{
                responsive: true,
                scales: {{
                    y: {{
                        min: 0,
                        max: 1.0,
                        ticks: {{
                            callback: function(value) {{
                                return (value * 100) + '%';
                            }},
                            color: '#94a3b8'
                        }},
                        grid: {{ color: '#334155' }}
                    }},
                    x: {{
                        title: {{ display: true, text: 'maxSteps', color: '#94a3b8' }},
                        ticks: {{ color: '#94a3b8' }},
                        grid: {{ color: '#334155' }}
                    }}
                }},
                plugins: {{
                    legend: {{ labels: {{ color: '#f8fafc' }} }}
                }}
            }}
        }});

        // Planning Time Chart
        const timeCtx = document.getElementById('timeChart').getContext('2d');
        new Chart(timeCtx, {{
            type: 'line',
            data: {{
                labels: labels,
                datasets: [{{
                    label: 'Mean Planning Time (ms)',
                    data: summaryData.map(d => d.mean_plan_time_ms),
                    borderColor: '#f43f5e',
                    backgroundColor: 'rgba(244, 63, 94, 0.1)',
                    tension: 0.1,
                    fill: true,
                    borderWidth: 3,
                    pointRadius: 6,
                    pointBackgroundColor: '#f43f5e'
                }}]
            }},
            options: {{
                responsive: true,
                scales: {{
                    y: {{
                        title: {{ display: true, text: 'ms per tick', color: '#94a3b8' }},
                        ticks: {{ color: '#94a3b8' }},
                        grid: {{ color: '#334155' }}
                    }},
                    x: {{
                        title: {{ display: true, text: 'maxSteps', color: '#94a3b8' }},
                        ticks: {{ color: '#94a3b8' }},
                        grid: {{ color: '#334155' }}
                    }}
                }},
                plugins: {{
                    legend: {{ labels: {{ color: '#f8fafc' }} }}
                }}
            }}
        }});
    </script>
</body>
</html>
"""
    html_file = "performance_dashboard.html"
    with open(html_file, "w") as f:
        f.write(html_content)
    print(f"Interactive HTML dashboard saved to {html_file}")

if __name__ == "__main__":
    try:
        create_headless_source()
        compile_headless()
        raw_data = run_benchmark()
        summary_stats = compute_statistics(raw_data)
        generate_html_dashboard(summary_stats)
    finally:
        # Cleanup temporary files
        if os.path.exists("main_headless.c"):
            os.remove("main_headless.c")
        if os.path.exists("main_headless"):
            os.remove("main_headless")
        print("Cleanup of temporary files completed.")

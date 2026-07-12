"""已弃用的 assignment-only 接口。

正式模式必须调用 scheduler.solve_assignment_schedule；保留该模块只为给旧调用者
提供明确错误，防止静默退化为仅分配不排序。
"""


def solve_assignment_only(*args, **kwargs):
    raise RuntimeError(
        "assignment_only is disabled in formal mode; use assignment_schedule"
    )

from rcbranch.config import load_config


def main():
    config = load_config()
    horizon = config.mpc.get("horizon_steps", "unknown")
    print(f"rcbranch planner package ready; horizon_steps={horizon}")


if __name__ == "__main__":
    main()

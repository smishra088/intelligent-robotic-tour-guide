def extract_preferences(text):
    prefs = {}
    text = text.lower()

    if "modern art" in text:
        prefs["tour_theme"] = "modern art"
    elif "sculpture" in text or "sculptures" in text:
        prefs["tour_theme"] = "sculpture"
    elif "painting" in text or "paintings" in text:
        prefs["tour_theme"] = "paintings"
    elif "digital" in text or "interactive" in text:
        prefs["tour_theme"] = "digital/interactive"

    if "avoid crowd" in text or "skip crowded" in text or "avoid busy" in text or "not too crowded" in text:
        prefs["avoid_crowds"] = True
    if "peaceful" in text or "quiet" in text:
        prefs["ambience"] = "peaceful"

    return prefs

if __name__ == "__main__":
    print("Preference Extraction Test (type 'exit' to stop)\n")
    while True:
        user_input = input("Enter museum command: ")
        if user_input.lower() == "exit":
            break
        preferences = extract_preferences(user_input)
        print(f"Extracted Preferences: {preferences}\n")


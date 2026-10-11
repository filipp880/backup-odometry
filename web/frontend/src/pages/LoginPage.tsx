import { useState } from "react";
import { useNavigate } from "react-router-dom";
import { authApi } from "../api/client";
import "./LoginPage.css";

export default function LoginPage() {
  const nav = useNavigate();
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [err, setErr] = useState("");
  const [mode, setMode] = useState<"login" | "register">("login");

  const submit = async (e: React.FormEvent) => {
    e.preventDefault();
    setErr("");
    try {
      const res = mode === "login"
        ? await authApi.login({ username, password })
        : await authApi.register({ username, password });
      localStorage.setItem("token", res.token);
      nav("/runs");
    } catch (e: any) {
      setErr(e.message || "Ошибка");
    }
  };

  return (
    <div className="login-wrap">
      <div className="login-card">
        <h2>{mode === "login" ? "Вход" : "Регистрация"}</h2>
        {err && <p className="err">{err}</p>}
        <form onSubmit={submit}>
          <label>Логин<input value={username} onChange={(e) => setUsername(e.target.value)} required /></label>
          <label>Пароль<input type="password" value={password} onChange={(e) => setPassword(e.target.value)} required /></label>
          <button type="submit">{mode === "login" ? "Войти" : "Зарегистрироваться"}</button>
        </form>
        <p>
          {mode === "login" ? "Нет аккаунта?" : "Уже есть аккаунт?"}{" "}
          <button type="button" onClick={() => setMode(mode === "login" ? "register" : "login")}>
            {mode === "login" ? "Регистрация" : "Войти"}
          </button>
        </p>
        <p className="demo">Демо: <code>admin / admin123</code> или <code>user / user123</code></p>
      </div>
    </div>
  );
}